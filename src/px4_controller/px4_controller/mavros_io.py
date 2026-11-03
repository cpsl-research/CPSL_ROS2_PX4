"""MAVROS backend of px4_control_node: MAVLink (e.g. the Pixhawk's USB) through MAVROS.

Implements the same console-facing interface as the XRCE backend; only the
transport differs. Everything below is relative to `<mavros_namespace>`
(default `mavros`) inside the node's namespace:

  in   state (mavros_msgs/State), sys_status (mavros_msgs/SysStatus),
       battery (sensor_msgs/BatteryState), local_position/odom (nav_msgs/Odometry, ENU)
  out  setpoint_raw/local (mavros_msgs/PositionTarget), streamed at 20 Hz from
       the first odometry on, services cmd/arming and set_mode

The setpoint stream is the OFFBOARD heartbeat over MAVLink (there is no
OffboardControlMode message): it runs continuously, holding the current
position when the core has no setpoint, and OFFBOARD is requested only once it
has run for 1 s (mavros_logic.OffboardStarter). Frames are left to MAVROS:
see mavros_logic.position_target.
"""
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State, SysStatus, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

from px4_controller import control_logic as cl
from px4_controller import mavros_logic as ml

STREAM_PERIOD_S = 0.05   # 20 Hz; PX4 needs > 2 Hz, the directive >= 10 Hz


class MavrosIO:
    name = 'mavros'

    def __init__(self, core):
        self.core = core
        ns = core.declare_parameter('mavros_namespace', 'mavros').value.strip('/')
        self.alt_hold_gain_per_s = float(core.declare_parameter('alt_hold_gain_per_s', 1.0).value)
        if not self.alt_hold_gain_per_s >= 0.0:
            raise ValueError('alt_hold_gain_per_s must be >= 0, got {}'.format(self.alt_hold_gain_per_s))

        def t(name):
            return '{}/{}'.format(ns, name) if ns else name

        # MAVROS publishes state/sys_status RELIABLE and odom/battery
        # BEST_EFFORT (SensorDataQoS); a BEST_EFFORT + VOLATILE subscriber
        # matches both.
        qos_in = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST, depth=5)
        # MAVROS subscribes setpoint_raw/local BEST_EFFORT; a RELIABLE
        # publisher matches it (and a reliable subscriber, should that change).
        qos_sp = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST, depth=1)

        core.create_subscription(State, t('state'), self.state_callback, qos_in)
        core.create_subscription(SysStatus, t('sys_status'), self.sys_status_callback, qos_in)
        core.create_subscription(BatteryState, t('battery'), self.battery_callback, qos_in)
        core.create_subscription(Odometry, t('local_position/odom'), self.odom_callback, qos_in)
        self.setpoint_publisher = core.create_publisher(PositionTarget, t('setpoint_raw/local'), qos_sp)
        self.arming_client = core.create_client(CommandBool, t('cmd/arming'))
        self.set_mode_client = core.create_client(SetMode, t('set_mode'))

        self.state = None               # last mavros_msgs/State
        self.ever_connected = False
        self.sensors_health = 0         # SYS_STATUS health bits (PREARM)
        self.current_enu = None
        self.setpoint = None            # last core Setpoint
        self.commanding = False         # the core's command stream is active
        self.hold = ml.HoldTracker()
        self.offboard = ml.OffboardStarter()
        self.stream_timer = core.create_timer(STREAM_PERIOD_S, self.stream_tick)
        core.get_logger().info('MAVROS backend on {}/ (setpoint stream {:.0f} Hz)'.format(
            ns, 1.0 / STREAM_PERIOD_S))

    # ------------------------------------------------------------------ #
    # Core -> MAVROS
    # ------------------------------------------------------------------ #

    def set_setpoint(self, sp: cl.Setpoint):
        self.setpoint = sp

    def activate_stream(self, period_s):
        """The stream always runs; this only selects the core's setpoint over the hold."""
        self.commanding = True

    def deactivate_stream(self):
        self.commanding = False

    def arm(self):
        return self._call(self.arming_client, CommandBool.Request(value=True), 'arm')

    def disarm(self):
        self.offboard.cancel_takeoff()
        return self._call(self.arming_client, CommandBool.Request(value=False), 'disarm')

    def land(self):
        self.offboard.cancel_takeoff()
        self._set_mode(ml.LAND, 'land')

    def on_takeoff(self):
        """Takeoff is an OFFBOARD position climb: make sure OFFBOARD is (re)requested."""
        self.offboard.request_takeoff()

    def _set_mode(self, mode, why):
        return self._call(self.set_mode_client, SetMode.Request(base_mode=0, custom_mode=mode),
                          'set_mode {} ({})'.format(mode, why))

    def _call(self, client, request, what):
        if not client.service_is_ready():
            self.core.get_logger().error('{}: MAVROS service {} not available'.format(
                what, client.srv_name))
            return False
        future = client.call_async(request)
        future.add_done_callback(lambda f: self._log_result(f, what))
        return True

    def _log_result(self, future, what):
        try:
            r = future.result()
        except Exception as e:  # noqa: BLE001 (any failure is only logged)
            self.core.get_logger().error('{}: call failed: {}'.format(what, e))
            return
        ok = getattr(r, 'mode_sent', getattr(r, 'success', None))
        self.core.get_logger().info('{}: {}'.format(what, 'accepted' if ok else 'REJECTED ({})'.format(r)))

    # ------------------------------------------------------------------ #
    # The setpoint stream (OFFBOARD heartbeat)
    # ------------------------------------------------------------------ #

    def _armed(self):
        return bool(self.state is not None and self.state.armed)

    def stream_tick(self):
        now_s = time.monotonic()
        armed = self._armed()
        hold = self.hold.update(armed, self.commanding and self.setpoint is not None, self.current_enu)
        what = ml.stream_setpoint(armed, self.commanding, self.setpoint, hold)
        if what == 'setpoint':
            current_alt = None if self.current_enu is None else float(self.current_enu[2])
            fields = ml.position_target(self.setpoint, current_alt,
                                        self.core.limits.max_climb_rate_mps, self.alt_hold_gain_per_s)
        elif what == 'hold':
            fields = ml.hold_target(hold)
        else:
            return  # no odometry yet: nothing to hold, so nothing is streamed
        self.setpoint_publisher.publish(self._msg(fields))
        self.offboard.note_sent(now_s)

        connected = bool(self.state is not None and self.state.connected)
        mode = self.state.mode if self.state is not None else ''
        reason = self.offboard.poll(now_s, connected, armed, mode)
        if reason:
            self._set_mode(ml.OFFBOARD, '{} after {:.1f} s of setpoints'.format(
                reason, self.offboard.streaming_s(now_s)))

    def _msg(self, f):
        msg = PositionTarget()
        msg.header.stamp = self.core.get_clock().now().to_msg()
        msg.coordinate_frame = f['coordinate_frame']
        msg.type_mask = f['type_mask']
        msg.position.x, msg.position.y, msg.position.z = f['position']
        msg.velocity.x, msg.velocity.y, msg.velocity.z = f['velocity']
        msg.yaw = f['yaw']
        msg.yaw_rate = f['yaw_rate']
        return msg

    # ------------------------------------------------------------------ #
    # MAVROS -> core
    # ------------------------------------------------------------------ #

    def _publish_state(self):
        s = self.state
        if s is None or not self.ever_connected:
            return  # nothing heard from PX4 yet: publish nothing rather than a guess
        self.core.on_vehicle_state(cl.VehicleState(
            armed=bool(s.armed),
            mode=ml.mode_name(s.mode),
            preflight_ok=ml.preflight_ok(self.sensors_health),
            failsafe=ml.failsafe(s.system_status),
        ))

    def state_callback(self, msg: State):
        if msg.connected and not self.ever_connected:
            self.core.get_logger().info('MAVROS connected to the FCU (mode {})'.format(msg.mode))
        self.ever_connected = self.ever_connected or bool(msg.connected)
        self.state = msg
        self._publish_state()

    def sys_status_callback(self, msg: SysStatus):
        self.sensors_health = int(msg.sensors_health)
        self._publish_state()

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.current_enu = (p.x, p.y, p.z)
        self.core.current_position_ned = ml.enu_to_ned(self.current_enu)
        self.core.current_q_ned = ml.q_enu_to_ned_wxyz([o.x, o.y, o.z, o.w])
        odom = Odometry()
        odom.header.stamp = self.core.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose = msg.pose
        odom.twist = msg.twist   # MAVROS: body (base_link) frame
        try:
            self.core.on_odometry(odom)
        except Exception as e:
            self.core.get_logger().error(f'Error in odom_callback: {str(e)}')

    def battery_callback(self, msg: BatteryState):
        # BATTERY_STATUS arrives at ~0.5 Hz: slow, not a link loss. The core
        # republishes the last value at 2 Hz.
        msg.voltage, msg.current, msg.percentage = ml.battery_fixups(
            msg.voltage, msg.current, msg.percentage)
        self.core.on_battery(msg, (bool(msg.present), int(msg.power_supply_health)))
