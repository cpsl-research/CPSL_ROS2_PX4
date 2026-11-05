"""XRCE-DDS backend of px4_control_node: PX4 uORB topics under /fmu/.

This is the original transport code of px4_control_node, moved here
unchanged in behaviour:

  - OffboardControlMode (position + velocity) at 5 Hz from start, and one
    nothing else on its own: OFFBOARD (VEHICLE_CMD_DO_SET_MODE) is requested only
    for an arm request or a take-off, once preflight_ok ("can fly OFFBOARD now",
    control_logic.offboard_ready) and the setpoint stream has run > 1 s;
  - the core's setpoint republished as TrajectorySetpoint at 20 Hz while the
    core's command stream is active (takeoff until arm/disarm/land);
  - VehicleCommand for arm, disarm and land;
  - /fmu/out/vehicle_status, failsafe_flags, vehicle_odometry and battery_status
    mapped to the core's status/*, odom and battery. preflight_ok is NOT PX4's
    pre_flight_checks_pass (which also needs RC, mag, GPS ...): that flag is
    published as status/px4_preflight_raw, the set failsafe flags as
    status/failsafe_flags;
  - additive telemetry (README "Console interface"): /fmu/out/vehicle_gps_position
    (SensorGps) -> gps/fix, vehicle_local_position.dist_bottom -> range, and
    vehicle_local_position + estimator_status_flags + vehicle_gps_position +
    battery warning + failsafe flags -> status/health. v1.15.4 publishes no
    distance_sensor topic over DDS, hence dist_bottom.

Topics are absolute (/fmu/...): one PX4 per DDS domain.
"""
import time

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (VehicleCommand, OffboardControlMode, TrajectorySetpoint,
                          VehicleStatus, VehicleOdometry, BatteryStatus, FailsafeFlags,
                          SensorGps, VehicleLocalPosition, EstimatorStatusFlags)
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

from px4_controller import control_logic as cl
from px4_controller import telemetry_logic as tl

OFFBOARD_HEARTBEAT_PERIOD_S = 0.2   # OffboardControlMode, 5 Hz


def px4_qos():
    """PX4's uXRCE bridge publishes BEST_EFFORT; unchanged from the original node."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )


class XrceIO:
    name = 'xrce'
    provides_telemetry = True    # gps/fix, range, status/health (mavros: not provided)

    def __init__(self, core):
        self.core = core
        qos_profile = px4_qos()

        self.vehicle_command_publisher = core.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_publisher = core.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = core.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        core.create_subscription(
            msg_type=VehicleStatus, topic='/fmu/out/vehicle_status',
            callback=self.vehicle_status_callback, qos_profile=qos_profile)
        core.create_subscription(
            msg_type=VehicleOdometry, topic='/fmu/out/vehicle_odometry',
            callback=self.vehicle_odometry_callback, qos_profile=qos_profile)
        core.create_subscription(
            msg_type=BatteryStatus, topic='/fmu/out/battery_status',
            callback=self.battery_status_callback, qos_profile=qos_profile)

        core.create_subscription(
            msg_type=FailsafeFlags, topic='/fmu/out/failsafe_flags',
            callback=self.failsafe_flags_callback, qos_profile=qos_profile)

        core.create_subscription(
            msg_type=SensorGps, topic='/fmu/out/vehicle_gps_position',
            callback=self.gps_callback, qos_profile=qos_profile)
        core.create_subscription(
            msg_type=VehicleLocalPosition, topic='/fmu/out/vehicle_local_position',
            callback=self.local_position_callback, qos_profile=qos_profile)
        core.create_subscription(
            msg_type=EstimatorStatusFlags, topic='/fmu/out/estimator_status_flags',
            callback=self.estimator_flags_callback, qos_profile=qos_profile)

        self.nav_state_names = cl.nav_state_names(VehicleStatus)
        self.last_status = None          # last VehicleStatus
        self.flag_names = None           # set failsafe flag names; None until heard
        self.requester = cl.OffboardRequester()
        self.guard = cl.DisarmGuard()
        self.prestream = False           # neutral hold setpoint while arming / armed idle
        self.stream_since_s = None
        self.active_command = None
        self.active_command_timer = None
        self.armed_seen = False
        self.timer = core.create_timer(OFFBOARD_HEARTBEAT_PERIOD_S, self.publish_offboard_control)

    # ------------------------------------------------------------------ #
    # OFFBOARD heartbeat and setpoints
    # ------------------------------------------------------------------ #

    def publish_offboard_control(self):
        offboard_control_mode = OffboardControlMode()
        offboard_control_mode.timestamp = self.core.get_clock().now().nanoseconds // 1000
        offboard_control_mode.position = True
        offboard_control_mode.velocity = True
        offboard_control_mode.acceleration = False
        offboard_control_mode.attitude = False
        offboard_control_mode.body_rate = False
        self.offboard_control_mode_publisher.publish(offboard_control_mode)

        now_s = time.monotonic()
        if self.prestream and self.active_command_timer is None:
            self._publish_hold_setpoint()
        streaming = self.prestream or self.active_command_timer is not None
        if streaming and self.stream_since_s is None:
            self.stream_since_s = now_s
        elif not streaming:
            self.stream_since_s = None
        streaming_s = 0.0 if self.stream_since_s is None else now_s - self.stream_since_s

        st = self.core.vehicle_state
        action = self.requester.poll(now_s, st.mode, st.armed, st.preflight_ok, streaming_s)
        if st.armed:
            self.armed_seen = True
        elif self.armed_seen:
            self.armed_seen = False      # disarmed again (landed): stop the hold stream
            self.prestream = False
        if action == 'offboard':
            self._enable_offboard_control()
        elif action == 'arm':
            self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.core.get_logger().info("Sent arming command (OFFBOARD active)")
        elif action == 'safe':
            self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0,
                                   param2=cl.SAFE_MAIN_MODE, param3=cl.SAFE_SUB_MODE)
            self.core.get_logger().info("Arm failed in OFFBOARD: requested AUTO_LOITER (hold)")
        if self.guard.poll(now_s, st.mode, st.armed, suppress=self.requester.active):
            self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0,
                                   param2=cl.SAFE_MAIN_MODE, param3=cl.SAFE_SUB_MODE)
            self.core.get_logger().info("Disarmed in OFFBOARD: requested AUTO_LOITER (hold)")

    def _publish_hold_setpoint(self):
        """Neutral OFFBOARD setpoint: hold the current position (NaN where unknown)."""
        pos = self.core.current_position_ned
        if pos is None:
            return   # no odometry: nothing to hold, no stream (arming waits)
        sp = TrajectorySetpoint()
        sp.timestamp = self.core.get_clock().now().nanoseconds // 1000
        sp.position = np.asarray(pos, dtype=np.float32)
        nan3 = np.array([float('nan')] * 3, dtype=np.float32)
        sp.velocity = nan3
        sp.acceleration = nan3
        sp.yaw = float('nan')
        sp.yawspeed = float('nan')
        self.trajectory_setpoint_publisher.publish(sp)

    def set_setpoint(self, sp: cl.Setpoint):
        trajectory_setpoint = TrajectorySetpoint()
        trajectory_setpoint.timestamp = self.core.get_clock().now().nanoseconds // 1000
        trajectory_setpoint.position = np.asarray(sp.position_ned, dtype=np.float32)
        trajectory_setpoint.velocity = np.asarray(sp.velocity_ned, dtype=np.float32)
        trajectory_setpoint.acceleration = np.array([0, 0, 0], dtype=np.float32)
        trajectory_setpoint.yaw = float(sp.yaw_rad)
        trajectory_setpoint.yawspeed = float(sp.yaw_speed)
        self.active_command = trajectory_setpoint

    def activate_stream(self, period_s):
        if self.active_command_timer is None:
            self.active_command_timer = self.core.create_timer(period_s, self._publish_active_command)
            self.core.get_logger().info("Activate active command timer")

    def deactivate_stream(self):
        if self.active_command_timer is not None:
            self.active_command_timer.cancel()
            self.active_command_timer = None

    def _publish_active_command(self):
        if self.active_command:
            self.active_command.timestamp = self.core.get_clock().now().nanoseconds // 1000
            self.trajectory_setpoint_publisher.publish(self.active_command)

    # ------------------------------------------------------------------ #
    # Vehicle commands
    # ------------------------------------------------------------------ #

    def arm(self):
        """Start the arm sequence: stream a hold setpoint, get into OFFBOARD (needs no
        RC), then arm. Sent by poll() in the 5 Hz timer. False if there is no
        odometry to hold."""
        if self.core.current_position_ned is None:
            self.core.get_logger().info("Arm not started: no odometry from PX4 yet")
            return False
        self.prestream = True
        self.requester.start_arm(time.monotonic())
        return True

    def disarm(self):
        self.requester.cancel()
        self.prestream = False
        self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        return True

    def land(self):
        self.requester.cancel()
        self.prestream = False
        self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def on_takeoff(self):
        """Take-off flies OFFBOARD: request it (once preflight_ok and streaming > 1 s) if not in it."""
        self.requester.start_takeoff(time.monotonic())

    def _enable_offboard_control(self):
        self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.core.get_logger().info("Sent offboard control command")

    def _send_vehicle_cmd(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.core.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    # ------------------------------------------------------------------ #
    # PX4 -> core
    # ------------------------------------------------------------------ #

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.last_status = msg
        self._publish_state()

    def gps_callback(self, msg):
        self.core.on_gps_fix(tl.gps_fix_fields(msg), tl.gps_health(msg))

    def local_position_callback(self, msg):
        self.core.on_range(tl.range_fields(msg.dist_bottom, msg.dist_bottom_valid))
        self.core.on_health_input('ekf', tl.ekf_validity(msg))

    def estimator_flags_callback(self, msg):
        self.core.on_health_input('fusion', tl.fusion_flags(msg))

    def failsafe_flags_callback(self, msg):
        self.flag_names = cl.set_failsafe_flag_names(msg)
        self.core.on_health_input('failsafe_flags', self.flag_names)
        self._publish_state()

    def _publish_state(self):
        msg = self.last_status
        if msg is None:
            return   # nothing heard from PX4 yet: publish nothing rather than a guess
        self.core.on_vehicle_state(cl.VehicleState(
            armed=msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            mode=cl.nav_state_name(msg.nav_state, self.nav_state_names),
            preflight_ok=cl.offboard_ready(msg.can_set_nav_states_mask, self.flag_names),
            failsafe=bool(msg.failsafe),
            failsafe_flags=self.flag_names or (),
            preflight_raw=bool(msg.pre_flight_checks_pass),
        ))

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        self.core.current_position_ned = msg.position
        self.core.current_q_ned = msg.q
        try:
            self.core.on_odometry(self.convert_px4_odometry_to_nav2(msg))
        except Exception as e:
            self.core.get_logger().error(f"Error in vehicle_odometry_callback: {str(e)}")

    def battery_status_callback(self, msg: BatteryStatus):
        state = BatteryState()
        for name, value in cl.battery_fields(msg).items():
            setattr(state, name, value)
        state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        state.serial_number = str(msg.serial_number)
        self.core.on_health_input('battery_warning', int(msg.warning))
        # published at once on the first message or a connection/warning change
        self.core.on_battery(state, (bool(msg.connected), int(msg.warning)))

    def convert_px4_odometry_to_nav2(self, vehicle_odom: VehicleOdometry) -> Odometry:
        """PX4 VehicleOdometry (NED/FRD, q [w,x,y,z]) -> nav_msgs/Odometry.

        World frame odom: NED turned 180 deg about x, i.e. x north, y west, z
        up (NWU; REP-105 gives odom no fixed compass direction). base_link is
        FLU (x forward, y left, z up). Orientation is the world rotation on
        the left and the FRD->FLU body rotation on the right:
        q_out = Rx180 * q_ned * Rx180. The twist is body FLU.
        """
        R_flip = Rotation.from_euler('x', 180, degrees=True)  # NED<->NWU and FRD<->FLU

        pos_ned = np.array([vehicle_odom.position[0], vehicle_odom.position[1], vehicle_odom.position[2]])
        pos_flu = R_flip.apply(pos_ned)

        px4_q = vehicle_odom.q  # [w, x, y, z], body FRD -> world NED
        quat_ned = [px4_q[1], px4_q[2], px4_q[3], px4_q[0]]  # [x, y, z, w]
        rot_ned = Rotation.from_quat(quat_ned)
        rot_flu = R_flip * rot_ned * R_flip
        quat_flu = rot_flu.as_quat()

        # velocity is in velocity_frame (NED expected); angular_velocity is
        # already body FRD, so only the FRD->FLU flip applies to it.
        vframe = getattr(vehicle_odom, 'velocity_frame', None)
        ned_const = getattr(VehicleOdometry, 'VELOCITY_FRAME_NED', None)
        if vframe is not None and ned_const is not None and vframe != ned_const:
            self.core.get_logger().error(
                f"VehicleOdometry velocity_frame={vframe} is not NED; velocity may be wrong")
        v_ned = np.array(vehicle_odom.velocity)
        omega_frd = np.array(vehicle_odom.angular_velocity)
        v_body = R_flip.apply(rot_ned.inv().apply(v_ned))
        omega_body = R_flip.apply(omega_frd)

        nav2_odom = Odometry()
        nav2_odom.header.stamp = self.core.get_clock().now().to_msg()
        nav2_odom.header.frame_id = "odom"
        nav2_odom.child_frame_id = "base_link"
        nav2_odom.pose.pose.position.x = pos_flu[0]
        nav2_odom.pose.pose.position.y = pos_flu[1]
        nav2_odom.pose.pose.position.z = pos_flu[2]
        nav2_odom.pose.pose.orientation.x = quat_flu[0]
        nav2_odom.pose.pose.orientation.y = quat_flu[1]
        nav2_odom.pose.pose.orientation.z = quat_flu[2]
        nav2_odom.pose.pose.orientation.w = quat_flu[3]
        nav2_odom.twist.twist.linear.x = v_body[0]
        nav2_odom.twist.twist.linear.y = v_body[1]
        nav2_odom.twist.twist.linear.z = v_body[2]
        nav2_odom.twist.twist.angular.x = omega_body[0]
        nav2_odom.twist.twist.angular.y = omega_body[1]
        nav2_odom.twist.twist.angular.z = omega_body[2]
        return nav2_odom
