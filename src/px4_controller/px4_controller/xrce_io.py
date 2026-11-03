"""XRCE-DDS backend of px4_control_node: PX4 uORB topics under /fmu/.

This is the original transport code of px4_control_node, moved here
unchanged in behaviour:

  - OffboardControlMode (position + velocity) at 5 Hz from start, and one
    VEHICLE_CMD_DO_SET_MODE offboard after the 10th heartbeat (2 s);
  - the core's setpoint republished as TrajectorySetpoint at 20 Hz while the
    core's command stream is active (takeoff until arm/disarm/land);
  - VehicleCommand for arm, disarm and land;
  - /fmu/out/vehicle_status, vehicle_odometry and battery_status mapped to the
    core's status/*, odom and battery.

Topics are absolute (/fmu/...): one PX4 per DDS domain.
"""
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (VehicleCommand, OffboardControlMode, TrajectorySetpoint,
                          VehicleStatus, VehicleOdometry, BatteryStatus)
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

from px4_controller import control_logic as cl

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

        self.nav_state_names = cl.nav_state_names(VehicleStatus)
        self.offboard_setpoint_counter = 0
        self.active_command = None
        self.active_command_timer = None
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

        if self.offboard_setpoint_counter == 10:
            self._enable_offboard_control()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

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
        self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        return True

    def disarm(self):
        self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        return True

    def land(self):
        self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def on_takeoff(self):
        """Nothing to do: OFFBOARD was requested once at start."""

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
        self.core.on_vehicle_state(cl.VehicleState(
            armed=msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            mode=cl.nav_state_name(msg.nav_state, self.nav_state_names),
            preflight_ok=bool(msg.pre_flight_checks_pass),
            failsafe=bool(msg.failsafe),
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
        # published at once on the first message or a connection/warning change
        self.core.on_battery(state, (bool(msg.connected), int(msg.warning)))

    def convert_px4_odometry_to_nav2(self, vehicle_odom: VehicleOdometry) -> Odometry:
        """PX4 VehicleOdometry (NED, q [w,x,y,z]) -> nav_msgs/Odometry.

        The world frame is NED rotated 180 deg about x (x north, y west, z up);
        the twist is body FLU.
        """
        R_ned_to_flu = Rotation.from_euler('x', 180, degrees=True)

        pos_ned = np.array([vehicle_odom.position[0], vehicle_odom.position[1], vehicle_odom.position[2]])
        pos_flu = R_ned_to_flu.apply(pos_ned)

        px4_q = vehicle_odom.q  # [w, x, y, z]
        quat_ned = [px4_q[1], px4_q[2], px4_q[3], px4_q[0]]  # [x, y, z, w]
        rot_ned = Rotation.from_quat(quat_ned)
        rot_flu = R_ned_to_flu * rot_ned
        quat_flu = rot_flu.as_quat()

        v_ned = np.array(vehicle_odom.velocity)
        omega_ned = np.array(vehicle_odom.angular_velocity)
        v_ned_body = rot_ned.inv().apply(v_ned)
        omega_ned_body = rot_ned.inv().apply(omega_ned)
        v_body = R_ned_to_flu.apply(v_ned_body)
        omega_body = R_ned_to_flu.apply(omega_ned_body)

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
