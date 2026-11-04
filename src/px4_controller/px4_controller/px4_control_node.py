"""px4_control_node: the transport-free core of the PX4 console interface.

The core owns everything the console sees and every flight decision:
the console-facing subscribers and publishers (README "Console interface"),
the takeoff / hover / flying state, the deadman and command staleness
timeout, target_altitude_m and its limits, the yaw steps, and the FLU ->
vehicle-frame velocity. It talks to the flight controller only through an
I/O adapter chosen by the `backend` parameter:

  xrce    xrce_io.XrceIO      PX4 uORB over Micro XRCE-DDS (/fmu/...), default
  mavros  mavros_io.MavrosIO  MAVLink through MAVROS (e.g. over USB)

Adapter interface (both implement it):
  set_setpoint(control_logic.Setpoint)   latest setpoint from the core
  activate_stream(period_s) / deactivate_stream()   core command active or not
  arm() -> bool, disarm() -> bool, land(), on_takeoff()
and the adapter calls back on_vehicle_state(), on_odometry(), on_battery(),
and sets current_position_ned / current_q_ned (PX4 local NED, q [w,x,y,z]).
"""
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Range
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from px4_controller import control_logic as cl
from px4_controller import telemetry_logic as tl

# Loop periods of this node (not flight limits; those are parameters).
VEL_CMD_PERIOD_S = 0.05      # process_vel_cmd_callback / setpoint republish
STATUS_PERIOD_S = 0.5        # status/*, battery and status/health republish (2 Hz)
RANGE_PERIOD_S = 0.1         # range is published at most at 10 Hz
BACKENDS = ('xrce', 'mavros')


def make_io(backend, core):
    """The adapter for `backend`, imported lazily so neither backend needs the other's messages."""
    if backend == 'xrce':
        from px4_controller.xrce_io import XrceIO
        return XrceIO(core)
    if backend == 'mavros':
        from px4_controller.mavros_io import MavrosIO
        return MavrosIO(core)
    raise ValueError('backend must be one of {}, got {!r}'.format(BACKENDS, backend))


class PX4ControlNode(Node):
    def __init__(self):
        super().__init__('px4_control_node')

        # Flight limits: every one is a parameter (console.launch.py passes them).
        for name, default in cl.LIMIT_DEFAULTS.items():
            self.declare_parameter(name, float(default))
        self.declare_parameter('tf_rate_hz', 30.0)
        self.declare_parameter('backend', 'xrce')
        self.limits = cl.Limits(**{
            name: float(self.get_parameter(name).value) for name in cl.LIMIT_DEFAULTS
        }).validate()
        tf_rate_hz = float(self.get_parameter('tf_rate_hz').value)
        if not tf_rate_hz >= 10.0:
            raise ValueError('tf_rate_hz must be >= 10 (docs/TF_AND_TOPICS.md §6), got {}'.format(
                tf_rate_hz))
        self.tf_min_period_s = 1.0 / tf_rate_hz
        backend = str(self.get_parameter('backend').value)
        self.get_logger().info('limits: {} tf_rate_hz={} backend={}'.format(
            self.limits, tf_rate_hz, backend))

        # odom: BEST_EFFORT + TRANSIENT_LOCAL, as the PX4 side is (README).
        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Command subscribers: VOLATILE. A TRANSIENT_LOCAL subscriber never
        # matches a VOLATILE publisher (commands silently dropped), and a
        # latched arm/takeoff/cmd_vel must never replay to a restarted node.
        # BEST_EFFORT accepts both reliable and best-effort publishers.
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # State publishers: latching state is safe, and RELIABLE +
        # TRANSIENT_LOCAL matches every subscriber QoS.
        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Vehicle state shared with the adapter (PX4 local NED, q [w, x, y, z]).
        self.current_position_ned = None
        self.current_q_ned = None

        # The flight-controller transport.
        self.io = make_io(backend, self)

        # Publishers - ROS2 Nav2/odometry messages
        self.px4_to_nav_odom_publisher = self.create_publisher(Odometry, 'odom', qos_odom)

        # Publishers - console-facing state
        self.status_armed_publisher = self.create_publisher(Bool, 'status/armed', qos_state)
        self.status_mode_publisher = self.create_publisher(String, 'status/mode', qos_state)
        self.status_preflight_publisher = self.create_publisher(
            Bool, 'status/preflight_ok', qos_state)
        self.status_failsafe_publisher = self.create_publisher(Bool, 'status/failsafe', qos_state)
        # additive: the set PX4 failsafe flag names (JSON list) and PX4's own preflight flag
        self.status_failsafe_flags_publisher = self.create_publisher(
            String, 'status/failsafe_flags', qos_state)
        self.status_preflight_raw_publisher = self.create_publisher(
            Bool, 'status/px4_preflight_raw', qos_state)
        self.battery_publisher = self.create_publisher(BatteryState, 'battery', qos_state)

        # Additive telemetry, only for a backend that provides it (xrce; mavros
        # leaves the topics unadvertised). Streams use the odom QoS, health the state QoS.
        self.has_telemetry = bool(getattr(self.io, 'provides_telemetry', False))
        self.gps_fix_publisher = self.range_publisher = self.health_publisher = None
        if self.has_telemetry:
            self.gps_fix_publisher = self.create_publisher(NavSatFix, 'gps/fix', qos_odom)
            self.range_publisher = self.create_publisher(Range, 'range', qos_odom)
            self.health_publisher = self.create_publisher(
                DiagnosticArray, 'status/health', qos_state)
        self.health_inputs = {}      # name -> latest decoded value (telemetry_logic)
        self.rx_s = {}               # source -> time.monotonic() of last message
        self.next_range_due_s = None

        # Subscribers - ROS2 cmd/control commands (keyop, joy, console)
        self.armed_status_subscriber = self.create_subscription(
            msg_type=Bool,
            topic='armed_status',
            callback=self.arm_status_callback,
            qos_profile=qos_cmd
        )

        self.takeoff_subscriber = self.create_subscription(
            msg_type=Bool,
            topic="takeoff",
            callback=self.takeoff_callback,
            qos_profile=qos_cmd
        )

        self.land_subscriber = self.create_subscription(
            msg_type=Bool,
            topic="land",
            callback=self.land_callback,
            qos_profile=qos_cmd
        )

        # Latest commands plus their receipt time (time.monotonic()); a
        # command older than cmd_timeout_s is ignored and the aircraft hovers.
        self.latest_cmd_vel_msg: TwistStamped = None
        self.cmd_vel_rx_s = None
        self.cmd_vel_subscriber = self.create_subscription(
            msg_type=TwistStamped,
            topic="cmd_vel",
            callback=self.cmd_vel_callback,
            qos_profile=qos_cmd
        )

        self.latest_cmd_vel_nav_msg: TwistStamped = None
        self.cmd_vel_nav_rx_s = None
        self.cmd_vel_nav_subscriber = self.create_subscription(
            msg_type=TwistStamped,
            topic="cmd_vel_nav",
            callback=self.cmd_vel_nav_callback,
            qos_profile=qos_cmd
        )

        self.allow_nav_cmds_subscriber = self.create_subscription(
            msg_type=Bool,
            topic='allow_nav_cmds',
            callback=self.allow_nav_cmds_callback,
            qos_profile=qos_cmd
        )

        self.deadman_rx_s = None
        self.deadman_switch_subscriber = self.create_subscription(
            msg_type=Bool,
            topic="deadman_pressed",
            callback=self.deadman_pressed_callback,
            qos_profile=qos_cmd
        )

        self.rotate_subscriber = self.create_subscription(
            msg_type=Bool,
            topic="rotate",
            callback=self.rotate_callback,
            qos_profile=qos_cmd
        )

        self.hover_position_ned = None

        # Altitude target, metres up from the odom origin. Set at takeoff to
        # takeoff_altitude_m, moved only by integrating clamped linear.z, kept
        # in [min_altitude_m, max_altitude_m]; None while not airborne.
        self.target_altitude_m = None
        self.last_altitude_update_s = None

        # control flags
        self.deadman_pressed: bool = False
        self.allow_nav_cmds_flag: bool = False
        self.vehicle_state = cl.VehicleState(armed=False, mode='', preflight_ok=False, failsafe=False)
        self.have_vehicle_status: bool = False
        self.last_status_published = None
        self.battery_latest: BatteryState = None
        self.last_battery_key = None
        self.flying_flag: bool = False
        self.hovering_flag: bool = True
        self.last_vel_decision = None

        # Timers
        self.status_timer = self.create_timer(STATUS_PERIOD_S, self.publish_state)
        self.process_vel_cmds_timer = None
        self.rotation_step_timer = None
        self.takeoff_timer = None

        # tf (odom -> base_link -> base_footprint) is sent from the odometry
        # callback, rate-limited to tf_rate_hz.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.next_tf_due_s = None

        return

    ####################################################################################
    # Setpoints (encoded and sent by the adapter)
    ####################################################################################

    def publish_trajectory_setpoint(
            self,
            position_ned: np.ndarray = np.array([np.nan, np.nan, np.nan]),
            linear_ned: np.ndarray = np.array([0, 0, 0]),
            yaw_rad: float = np.nan,
            yaw_speed: float = 0.0,
            caller: str = "N/A",
            body_velocity_flu=None,
            climb_rate_mps: float = 0.0):

        if not self.vehicle_state.armed:
            self.get_logger().info("Failed to send position command from {} because not armed".format(
                caller
            ))
            return

        self.io.set_setpoint(cl.Setpoint(
            position_ned=position_ned,
            velocity_ned=linear_ned,
            yaw_rad=yaw_rad,
            yaw_speed=yaw_speed,
            body_velocity_flu=body_velocity_flu,
            climb_rate_mps=float(climb_rate_mps),
            target_altitude_m=(float('nan') if self.target_altitude_m is None
                               else float(self.target_altitude_m)),
            caller=caller,
        ))

    def activate_command_timer(self):
        self.io.activate_stream(VEL_CMD_PERIOD_S)

    def deactivate_command_timer(self):
        self.io.deactivate_stream()

    def activate_process_vel_cmds_timer(self):

        if self.process_vel_cmds_timer is None:
            self.process_vel_cmds_timer = self.create_timer(VEL_CMD_PERIOD_S, self.process_vel_cmd_callback)
            self.get_logger().info("Activate process velocity commands command timer")

    def deactivate_process_vel_cmds_timer(self):
        if self.process_vel_cmds_timer is not None:
            self.process_vel_cmds_timer.cancel()
            self.process_vel_cmds_timer = None

    ####################################################################################
    # Adapter -> core
    ####################################################################################

    def on_vehicle_state(self, state: cl.VehicleState):
        self.rx_s['vehicle_status'] = time.monotonic()
        self.vehicle_state = state
        self.have_vehicle_status = True
        # publish on change; publish_state also republishes at 2 Hz
        if self._status_tuple() != self.last_status_published:
            self.publish_status()

    def on_odometry(self, nav2_odometry: Odometry):
        """Publish odom (frame odom, child base_link) and, rate-limited, the tf from it."""
        self.px4_to_nav_odom_publisher.publish(nav2_odometry)
        self.rx_s['odometry'] = time.monotonic()
        send_tf, self.next_tf_due_s = cl.rate_limit(
            time.monotonic(), self.next_tf_due_s, self.tf_min_period_s)
        if send_tf:
            self.broadcast_tf(nav2_odometry)

    def on_battery(self, msg: BatteryState, key):
        self.battery_latest = msg
        # publish immediately on the first message or when key changes
        if key != self.last_battery_key:
            self.last_battery_key = key
            self.publish_battery()

    def on_gps_fix(self, fields: dict, health: dict):
        """gps/fix from telemetry_logic.gps_fix_fields; `health` feeds status/health."""
        self.health_inputs['gps'] = health
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = fields['status']
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = fields['latitude']
        msg.longitude = fields['longitude']
        msg.altitude = fields['altitude']
        msg.position_covariance = fields['covariance']
        msg.position_covariance_type = fields['covariance_type']
        self.gps_fix_publisher.publish(msg)

    def on_range(self, fields: dict):
        """Publish range (telemetry_logic.range_fields) at most every RANGE_PERIOD_S."""
        send, self.next_range_due_s = cl.rate_limit(
            time.monotonic(), self.next_range_due_s, RANGE_PERIOD_S)
        if not send:
            return
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = float('nan')
        msg.min_range = float('nan')
        msg.max_range = float('nan')
        msg.range = fields['range']
        self.range_publisher.publish(msg)

    def on_health_input(self, name: str, value):
        """Latest decoded input for status/health (see telemetry_logic.health_items)."""
        self.health_inputs[name] = value
        if name == 'ekf':
            self.rx_s['local_position'] = time.monotonic()

    def publish_health(self):
        if self.health_publisher is None or not self.have_vehicle_status:
            return
        now = time.monotonic()
        ages = {src: (None if self.rx_s.get(src) is None else now - self.rx_s[src])
                for src in ('vehicle_status', 'odometry', 'local_position')}
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for item in tl.health_items(self.health_inputs, ages):
            st = DiagnosticStatus()
            st.level = bytes([item['level']])
            st.name = 'px4/' + item['name']
            st.message = item['message']
            st.hardware_id = 'px4'
            st.values = [KeyValue(key=k, value=v) for k, v in item['values'].items()]
            msg.status.append(st)
        self.health_publisher.publish(msg)

    ####################################################################################
    # Console-facing state (status/*, battery)
    ####################################################################################

    def _status_tuple(self):
        if not self.have_vehicle_status:
            return None
        return self.vehicle_state.as_tuple()

    def publish_status(self):
        status = self._status_tuple()
        if status is None:
            return  # nothing heard from PX4 yet: publish nothing rather than a guess
        armed, mode, preflight_ok, failsafe, flags, raw = status
        self.status_armed_publisher.publish(Bool(data=armed))
        self.status_mode_publisher.publish(String(data=mode))
        self.status_preflight_publisher.publish(Bool(data=preflight_ok))
        self.status_failsafe_publisher.publish(Bool(data=failsafe))
        self.status_failsafe_flags_publisher.publish(String(data=json.dumps(list(flags))))
        if raw is not None:
            self.status_preflight_raw_publisher.publish(Bool(data=raw))
        self.last_status_published = status

    def publish_battery(self):
        if self.battery_latest is None:
            return
        msg = self.battery_latest
        msg.header.stamp = self.get_clock().now().to_msg()
        self.battery_publisher.publish(msg)

    def publish_state(self):
        self.publish_status()
        self.publish_battery()
        self.publish_health()

    ####################################################################################
    # tf
    ####################################################################################

    def broadcast_tf(self, nav2_odom: Odometry):
        """odom -> base_link, and base_link -> base_footprint (TF_AND_TOPICS §4).

        Frames are unprefixed; the namespace is carried by the tf topic
        (console.launch.py remaps /tf -> tf inside the namespace).
        """
        t = TransformStamped()
        t.header.stamp = nav2_odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = nav2_odom.pose.pose.position.x
        t.transform.translation.y = nav2_odom.pose.pose.position.y
        t.transform.translation.z = nav2_odom.pose.pose.position.z
        t.transform.rotation = nav2_odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform([t, self.base_footprint_tf(t)])

    def base_footprint_tf(self, odom_to_base_tf: TransformStamped) -> TransformStamped:
        """base_link -> base_footprint: base_link projected to odom z=0, yaw only."""
        tr = odom_to_base_tf.transform.translation
        rot = odom_to_base_tf.transform.rotation
        t_bf, q_bf = cl.base_link_to_footprint(
            [tr.x, tr.y, tr.z], [rot.x, rot.y, rot.z, rot.w])

        new_tf = TransformStamped()
        new_tf.header.stamp = odom_to_base_tf.header.stamp
        new_tf.header.frame_id = "base_link"
        new_tf.child_frame_id = "base_footprint"
        new_tf.transform.translation.x = float(t_bf[0])
        new_tf.transform.translation.y = float(t_bf[1])
        new_tf.transform.translation.z = float(t_bf[2])
        new_tf.transform.rotation.x = float(q_bf[0])
        new_tf.transform.rotation.y = float(q_bf[1])
        new_tf.transform.rotation.z = float(q_bf[2])
        new_tf.transform.rotation.w = float(q_bf[3])
        return new_tf

    ####################################################################################
    # Mode control commands
    ####################################################################################

    def _px4_send_arm_cmd(self) -> bool:
        """Arm (only when preflight_ok: PX4 can fly OFFBOARD now; the backend may first
        switch to OFFBOARD with setpoints streaming).

        Returns:
            bool: Command successfully sent
        """
        self.deactivate_process_vel_cmds_timer()
        self.deactivate_command_timer()

        if self.vehicle_state.preflight_ok:
            started = self.io.arm()
            self.get_logger().info("Arming requested" if started is not False
                                   else "Arming not started")
            return started is not False
        else:
            self.get_logger().info("Did not ARM because pre flight checks failed")
            return False

    def _px4_send_disarm_cmd(self):
        """Disarm.

        Returns:
            Bool: command sent successfully
        """
        self.deactivate_process_vel_cmds_timer()
        self.deactivate_command_timer()
        self.io.disarm()
        self.get_logger().info("Sent disarming command")

        return True

    def _cancel_rotation(self):
        if self.rotation_step_timer is not None:
            self.rotation_step_timer.cancel()
            self.destroy_timer(self.rotation_step_timer)
            self.rotation_step_timer = None

    def _px4_send_land_cmd(self):

        if self.takeoff_timer:
            self.destroy_timer(self.takeoff_timer)
            self.takeoff_timer = None
        self._cancel_rotation()

        self.deactivate_process_vel_cmds_timer()
        self.deactivate_command_timer()
        self.io.land()

        # TODO: Add code to check that landing was successful
        self.get_logger().info("Sent land command")

    def _px4_send_takeoff_cmd(self):

        if self.flying_flag or self.takeoff_timer is not None:
            self.get_logger().info("Takeoff ignored: already flying or taking off")
            return
        if self.current_position_ned is None:
            self.get_logger().info("Takeoff aborted: no odometry from PX4 yet")
            return

        # the altitude target starts at the takeoff altitude
        self.target_altitude_m = self.limits.takeoff_altitude_m
        self.last_altitude_update_s = None

        # set the hovering flag and position (xy where we took off)
        self.hovering_flag = True
        self.hover_position_ned = np.array(self.current_position_ned, dtype=float)

        # set the position for takeoff
        self.takeoff_position_ned = np.array(self.current_position_ned, dtype=float)
        self.takeoff_position_ned[2] = -self.target_altitude_m

        # start the takeoff timer
        self.takeoff_timer = self.create_timer(0.2, self.control_takeoff)
        self.get_logger().info("Sent takeoff command, target altitude {:.2f} m".format(
            self.target_altitude_m))

        # activate the command timer
        self.activate_command_timer()
        self.io.on_takeoff()

    def control_takeoff(self):
        if self.target_altitude_m is None:
            return
        current_altitude_m = -float(self.current_position_ned[2])

        # Takeoff is complete once inside the flight envelope (>= min_altitude_m);
        # the setpoint stays at takeoff_altitude_m (> min_altitude_m) and the
        # hover setpoint keeps driving there afterwards.
        if current_altitude_m >= self.limits.min_altitude_m:
            if self.takeoff_timer is not None:
                self.takeoff_timer.cancel()
                self.destroy_timer(self.takeoff_timer)
                self.takeoff_timer = None

            # set the flying flag to true
            self.flying_flag = True
            self.hovering_flag = True

            # start processing velocity commands
            self.activate_process_vel_cmds_timer()

        self.takeoff_position_ned[2] = -self.target_altitude_m
        self.publish_trajectory_setpoint(
            position_ned=self.takeoff_position_ned,
            linear_ned=np.array([0, 0, 0]),
            caller="takeoff"
        )

    ####################################################################################
    # ROS2 control commands
    ####################################################################################

    def arm_status_callback(self, msg: Bool):

        if msg.data:
            self._px4_send_arm_cmd()
        else:
            self._px4_send_disarm_cmd()

    def takeoff_callback(self, msg: Bool):
        if not msg.data:
            return  # only takeoff:=true takes off
        if self.vehicle_state.armed:
            self._px4_send_takeoff_cmd()
        else:
            self.get_logger().info("Takeoff aborted because not armed")

    def land_callback(self, msg: Bool):
        # any message on land lands (fail toward landing)
        # set the flying flag to false (disables velocity commands)
        self.flying_flag = False
        self.hovering_flag = True
        self.target_altitude_m = None
        self.last_altitude_update_s = None

        # then send the landing command
        self._px4_send_land_cmd()

    def cmd_vel_callback(self, msg: TwistStamped):
        self.latest_cmd_vel_msg = msg
        self.cmd_vel_rx_s = time.monotonic()

    def cmd_vel_nav_callback(self, msg: TwistStamped):
        self.latest_cmd_vel_nav_msg = msg
        self.cmd_vel_nav_rx_s = time.monotonic()

    def process_vel_cmd_callback(self):
        now_s = time.monotonic()
        timeout_s = self.limits.cmd_timeout_s
        decision = cl.select_command(
            flying=self.flying_flag,
            deadman_pressed=self.deadman_pressed,
            deadman_fresh=cl.is_fresh(self.deadman_rx_s, now_s, timeout_s),
            allow_nav=self.allow_nav_cmds_flag,
            cmd_vel_fresh=cl.is_fresh(self.cmd_vel_rx_s, now_s, timeout_s),
            cmd_vel_nav_fresh=cl.is_fresh(self.cmd_vel_nav_rx_s, now_s, timeout_s),
        )
        if decision != self.last_vel_decision:
            self.get_logger().info("velocity source: {}".format(decision))
            self.last_vel_decision = decision

        if decision == cl.USE_NAV:
            self.send_vel_cmd(self.latest_cmd_vel_nav_msg, "nav", now_s)
        elif decision == cl.USE_MANUAL:
            self.send_vel_cmd(self.latest_cmd_vel_msg, "manual", now_s)
        else:
            # stale, missing or blocked command: hold position and altitude.
            # The next fresh command restarts altitude integration from dt=0.
            self.last_altitude_update_s = None
            self.send_hover_trajectory_setpoint(target_yaw_speed=0.0)

    def send_vel_cmd(self, msg: TwistStamped, source_type: str, now_s: float):

        try:
            linear_flu = msg.twist.linear

            # altitude: integrate clamped linear.z into the target
            dt_s = 0.0 if self.last_altitude_update_s is None else now_s - self.last_altitude_update_s
            self.last_altitude_update_s = now_s
            self.target_altitude_m, climb_rate = cl.integrate_altitude(
                self.target_altitude_m, linear_flu.z, dt_s, self.limits,
                max_dt_s=2 * VEL_CMD_PERIOD_S)

            # lateral: body FLU x/y to NED
            linear_ned = cl.lateral_flu_to_ned(linear_flu.x, linear_flu.y, self.current_q_ned)

            angular = msg.twist.angular
            target_yaw_speed = -1 * angular.z

            # Check to see if the vehicle is in hovering state (zero lateral vels)
            if linear_flu.x == 0 and linear_flu.y == 0:
                self.send_hover_trajectory_setpoint(
                    target_yaw_speed=target_yaw_speed,
                    climb_rate_mps=climb_rate
                )
            else:
                self.hovering_flag = False

                self.publish_trajectory_setpoint(
                    position_ned=np.array([np.nan, np.nan, -self.target_altitude_m]),
                    linear_ned=np.array([linear_ned[0], linear_ned[1], -climb_rate]),
                    yaw_speed=target_yaw_speed,
                    caller="velocity commander",
                    body_velocity_flu=(float(linear_flu.x), float(linear_flu.y)),
                    climb_rate_mps=climb_rate,
                )

        except Exception as e:
            self.get_logger().info(f"Failed to parse velocity callback ({source_type}): {e}")

    def send_hover_trajectory_setpoint(self, target_yaw_speed, climb_rate_mps=0.0):
        """Hold xy (latched when hovering starts) at the current altitude target."""
        if self.target_altitude_m is None or self.current_position_ned is None:
            return

        # only update the hover position if the hovering flag was previously false
        if not self.hovering_flag or self.hover_position_ned is None:
            self.hovering_flag = True
            self.hover_position_ned = np.array(self.current_position_ned, dtype=float)

        target_position_ned = np.array(self.hover_position_ned, dtype=float)
        target_position_ned[2] = -self.target_altitude_m

        self.publish_trajectory_setpoint(
            position_ned=target_position_ned,
            linear_ned=np.array([0, 0, -climb_rate_mps]),
            yaw_speed=target_yaw_speed,
            caller="hovering",
            climb_rate_mps=climb_rate_mps,
        )

    def allow_nav_cmds_callback(self, msg: Bool):
        """Sets the flag for whether of not to allow for nav commands to
        autonomously control the UAV

        Args:
            msg (Bool): a ROS2 Bool message
        """
        self.allow_nav_cmds_flag = msg.data

    def deadman_pressed_callback(self, msg: Bool):
        """Sets the flag for whether of not the deadman switch is pressed.

        The value is only trusted for cmd_timeout_s after receipt, so a
        publisher must repeat it while the switch is held.

        Args:
            msg (Bool): a ROS2 Bool message
        """
        self.deadman_pressed = msg.data
        self.deadman_rx_s = time.monotonic()

    def rotate_callback(self, msg: Bool):

        if not self.flying_flag or self.target_altitude_m is None or self.current_q_ned is None:
            self.get_logger().info("rotate ignored: not flying")
            return

        q = self.current_q_ned
        r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        current_yaw = r.as_euler('zyx')[0]

        self.rotate_position = np.array(self.current_position_ned, dtype=float)
        self.rotate_position[2] = -self.target_altitude_m

        self.start_yaw = current_yaw
        if msg.data:
            self.target_yaw = (current_yaw - np.pi / 2) % (2 * np.pi) - np.pi
        else:
            self.target_yaw = (current_yaw + np.pi / 2) % (2 * np.pi) - np.pi

        self.yaw_step_size = np.deg2rad(4)
        self.current_yaw_step = 0

        self._cancel_rotation()

        # deactivate the command velocity commander
        self.deactivate_process_vel_cmds_timer()
        self.rotation_step_timer = self.create_timer(0.2, self.incremental_yaw_step)

    def incremental_yaw_step(self):
        yaw_diff = (self.target_yaw - self.start_yaw + np.pi) % (2 * np.pi) - np.pi

        num_steps = int(np.abs(yaw_diff) / self.yaw_step_size)

        if self.current_yaw_step > num_steps:
            self._cancel_rotation()
            if self.flying_flag:
                self.activate_process_vel_cmds_timer()
            return

        step_yaw = self.start_yaw + self.current_yaw_step * np.sign(yaw_diff) * self.yaw_step_size
        step_yaw = (step_yaw + np.pi) % (2 * np.pi) - np.pi

        self.publish_trajectory_setpoint(
            position_ned=self.rotate_position,
            yaw_rad=step_yaw,
            caller="yaw step updater"
        )
        self.current_yaw_step += 1


def main(args=None):
    rclpy.init(args=args)
    px4_control_node = PX4ControlNode()
    try:
        rclpy.spin(px4_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # SIGINT already shut the context down under ros2 launch
            rclpy.shutdown()


if __name__ == '__main__':
    main()
