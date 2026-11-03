import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (VehicleCommand, OffboardControlMode, TrajectorySetpoint,
                          VehicleStatus, VehicleOdometry, BatteryStatus)
from std_msgs.msg import String, Bool
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import BatteryState
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from px4_controller import control_logic as cl

# Loop periods of this node (not flight limits; those are parameters).
VEL_CMD_PERIOD_S = 0.05      # process_vel_cmd_callback / setpoint republish
STATUS_PERIOD_S = 0.5        # status/* and battery republish (2 Hz)


class PX4ControlNode(Node):
    def __init__(self):
        super().__init__('px4_control_node')

        # Flight limits: every one is a parameter (console.launch.py passes them).
        for name, default in cl.LIMIT_DEFAULTS.items():
            self.declare_parameter(name, float(default))
        self.declare_parameter('tf_rate_hz', 30.0)
        self.limits = cl.Limits(**{
            name: float(self.get_parameter(name).value) for name in cl.LIMIT_DEFAULTS
        }).validate()
        tf_rate_hz = float(self.get_parameter('tf_rate_hz').value)
        if not tf_rate_hz >= 10.0:
            raise ValueError('tf_rate_hz must be >= 10 (docs/TF_AND_TOPICS.md §6), got {}'.format(
                tf_rate_hz))
        self.tf_min_period_s = 1.0 / tf_rate_hz
        self.get_logger().info('limits: {} tf_rate_hz={}'.format(self.limits, tf_rate_hz))

        # PX4 side (/fmu/..., absolute: one PX4 per DDS domain). Unchanged from
        # the original node; PX4's uXRCE bridge publishes BEST_EFFORT.
        qos_profile = QoSProfile(
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

        # Publishers - PX4 messages
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # Publishers - ROS2 Nav2/odometry messages
        self.px4_to_nav_odom_publisher = self.create_publisher(
            Odometry, 'odom', qos_profile)

        # Publishers - console-facing state
        self.status_armed_publisher = self.create_publisher(Bool, 'status/armed', qos_state)
        self.status_mode_publisher = self.create_publisher(String, 'status/mode', qos_state)
        self.status_preflight_publisher = self.create_publisher(
            Bool, 'status/preflight_ok', qos_state)
        self.status_failsafe_publisher = self.create_publisher(Bool, 'status/failsafe', qos_state)
        self.battery_publisher = self.create_publisher(BatteryState, 'battery', qos_state)

        # Subscribers - PX4 messages
        self.vehicle_status_subscriber = self.create_subscription(
            msg_type=VehicleStatus,
            topic='/fmu/out/vehicle_status',
            callback=self.vehicle_status_callback,
            qos_profile=qos_profile
        )

        self.vehicle_odometry_subscriber = self.create_subscription(
            msg_type=VehicleOdometry,
            topic='/fmu/out/vehicle_odometry',
            callback=self.vehicle_odometry_callback,
            qos_profile=qos_profile
        )

        self.battery_status_subscriber = self.create_subscription(
            msg_type=BatteryStatus,
            topic='/fmu/out/battery_status',
            callback=self.battery_status_callback,
            qos_profile=qos_profile
        )

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

        self.current_position_ned = None
        self.current_q_ned = None

        self.hover_position_ned = None

        # Altitude target, metres up from the odom origin. Set at takeoff to
        # takeoff_altitude_m, moved only by integrating clamped linear.z, kept
        # in [min_altitude_m, max_altitude_m]; None while not airborne.
        self.target_altitude_m = None
        self.last_altitude_update_s = None

        self.offboard_setpoint_counter = 0

        # control flags
        self.deadman_pressed: bool = False
        self.allow_nav_cmds_flag: bool = False
        self.vehicle_status_latest: VehicleStatus = VehicleStatus()
        self.have_vehicle_status: bool = False
        self.nav_state_names = cl.nav_state_names(VehicleStatus)
        self.last_status_published = None
        self.battery_status_latest: BatteryStatus = None
        self.last_battery_key = None
        self.flying_flag: bool = False
        self.hovering_flag: bool = True
        self.last_vel_decision = None

        # Timers
        self.timer = self.create_timer(0.2, self.publish_offboard_control)
        self.status_timer = self.create_timer(STATUS_PERIOD_S, self.publish_state)
        self.active_command_timer = None
        self.active_command = None
        self.process_vel_cmds_timer = None
        self.rotation_step_timer = None
        self.takeoff_timer = None

        # tf (odom -> base_link -> base_footprint) is sent from the odometry
        # callback, rate-limited to tf_rate_hz.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.next_tf_due_s = None

        return

    def publish_offboard_control(self):
        # Publish OffboardControlMode
        offboard_control_mode = OffboardControlMode()
        offboard_control_mode.timestamp = self.get_clock().now().nanoseconds // 1000  # Timestamp in microseconds
        offboard_control_mode.position = True
        offboard_control_mode.velocity = True
        offboard_control_mode.acceleration = False
        offboard_control_mode.attitude = False
        offboard_control_mode.body_rate = False
        self.offboard_control_mode_publisher.publish(offboard_control_mode)

        if self.offboard_setpoint_counter == 10:
            self._px4_enable_offboard_control()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    ####################################################################################
    ####################################################################################
    # PX4 Trajectory control commands
    ####################################################################################
    ####################################################################################

    def publish_trajectory_setpoint(
            self,
            position_ned: np.ndarray = np.array([np.nan, np.nan, np.nan]),
            linear_ned: np.ndarray = np.array([0, 0, 0]),
            yaw_rad: float = np.nan,
            yaw_speed: float = 0.0,
            caller: str = "N/A"):

        if self.vehicle_status_latest.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("Failed to send position command from {} because not armed".format(
                caller
            ))
            return

        trajectory_setpoint = TrajectorySetpoint()
        trajectory_setpoint.timestamp = self.get_clock().now().nanoseconds // 1000
        trajectory_setpoint.position = np.asarray(position_ned, dtype=np.float32)
        trajectory_setpoint.velocity = np.asarray(linear_ned, dtype=np.float32)
        trajectory_setpoint.acceleration = np.array([0, 0, 0], dtype=np.float32)
        trajectory_setpoint.yaw = float(yaw_rad)
        trajectory_setpoint.yawspeed = float(yaw_speed)

        self.active_command = trajectory_setpoint

    def activate_command_timer(self):
        if self.active_command_timer is None:
            self.active_command_timer = self.create_timer(VEL_CMD_PERIOD_S, self._publish_active_command)
            self.get_logger().info("Activate active command timer")

    def deactivate_command_timer(self):
        if self.active_command_timer is not None:
            self.active_command_timer.cancel()
            self.active_command_timer = None

    def _publish_active_command(self):

        if self.active_command:
            self.active_command.timestamp = self.get_clock().now().nanoseconds // 1000
            self.trajectory_setpoint_publisher.publish(self.active_command)

    def activate_process_vel_cmds_timer(self):

        if self.process_vel_cmds_timer is None:
            self.process_vel_cmds_timer = self.create_timer(VEL_CMD_PERIOD_S, self.process_vel_cmd_callback)
            self.get_logger().info("Activate process velocity commands command timer")

    def deactivate_process_vel_cmds_timer(self):
        if self.process_vel_cmds_timer is not None:
            self.process_vel_cmds_timer.cancel()
            self.process_vel_cmds_timer = None

    ####################################################################################
    ####################################################################################
    # PX4 Subscriptions
    ####################################################################################
    ####################################################################################

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status_latest = msg
        self.have_vehicle_status = True
        # publish on change; publish_state also republishes at 2 Hz
        if self._status_tuple() != self.last_status_published:
            self.publish_status()

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        self.current_position_ned = msg.position
        self.current_q_ned = msg.q
        self.current_yaw_speed = msg.angular_velocity[2]

        try:
            # Store the latest odometry message
            self.vehicle_odometry_latest = msg
            # Convert and publish immediately
            nav2_odometry = self.convert_px4_odometry_to_nav2(msg)
            self.px4_to_nav_odom_publisher.publish(nav2_odometry)

            # tf from the same conversion and stamp, rate-limited
            send_tf, self.next_tf_due_s = cl.rate_limit(
                time.monotonic(), self.next_tf_due_s, self.tf_min_period_s)
            if send_tf:
                self.broadcast_tf(nav2_odometry)

        except Exception as e:
            self.get_logger().error(f"Error in vehicle_odometry_callback: {str(e)}")

    def battery_status_callback(self, msg: BatteryStatus):
        self.battery_status_latest = msg
        # publish immediately on the first message or a connection/warning change
        key = (bool(msg.connected), int(msg.warning))
        if key != self.last_battery_key:
            self.last_battery_key = key
            self.publish_battery()

    ####################################################################################
    ####################################################################################
    # Console-facing state (status/*, battery)
    ####################################################################################

    def _status_tuple(self):
        if not self.have_vehicle_status:
            return None
        s = self.vehicle_status_latest
        return (
            s.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            cl.nav_state_name(s.nav_state, self.nav_state_names),
            bool(s.pre_flight_checks_pass),
            bool(s.failsafe),
        )

    def publish_status(self):
        status = self._status_tuple()
        if status is None:
            return  # nothing heard from PX4 yet: publish nothing rather than a guess
        armed, mode, preflight_ok, failsafe = status
        self.status_armed_publisher.publish(Bool(data=armed))
        self.status_mode_publisher.publish(String(data=mode))
        self.status_preflight_publisher.publish(Bool(data=preflight_ok))
        self.status_failsafe_publisher.publish(Bool(data=failsafe))
        self.last_status_published = status

    def publish_battery(self):
        if self.battery_status_latest is None:
            return
        fields = cl.battery_fields(self.battery_status_latest)
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, value in fields.items():
            setattr(msg, name, value)
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.serial_number = str(self.battery_status_latest.serial_number)
        self.battery_publisher.publish(msg)

    def publish_state(self):
        self.publish_status()
        self.publish_battery()

    #####################################################################################
    ####################################################################################
    # PX4 Odometry Conversion
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

    def convert_px4_odometry_to_nav2(self, vehicle_odom: VehicleOdometry) -> Odometry:
        """
        Convert a real PX4 odometry message (VehicleOdometry in NED with quaternion [w,x,y,z])
        to a ROS2 nav_msgs/Odometry message in ENU. Convert from NED to FLU to ENU.
        """
        
        # Define the rotation matrices for coordinate frame conversions
        R_ned_to_flu = Rotation.from_euler('x',180,degrees=True)#Rotation.from_euler('x', 180, degrees=True)
        # R_flu_to_enu = Rotation.from_euler('z',0)#Rotation.from_euler('z', -90, degrees=True)
        
        # Convert position from NED to FLU to ENU
        pos_ned = np.array([vehicle_odom.position[0],  vehicle_odom.position[1],  vehicle_odom.position[2]])
        pos_flu = R_ned_to_flu.apply(pos_ned)
        # pos_enu = R_flu_to_enu.apply(pos_flu)
        
        #  quaternion from NED to ENU
        px4_q = vehicle_odom.q  # [w, x, y, z]
        quat_ned = [px4_q[1], px4_q[2], px4_q[3], px4_q[0]]  # [x, y, z, w]
        # Convert quaternion through the rotation chain
        rot_ned = Rotation.from_quat(quat_ned)
        rot_flu = R_ned_to_flu * rot_ned
        quat_flu = rot_flu.as_quat()
        # rot_enu = R_flu_to_enu * rot_flu
        # quat_enu = rot_enu.as_quat()  # Returns [x, y, z, w]
        
        # Convert velocities from NED to FLU (global FLU)
        v_ned = np.array(vehicle_odom.velocity)
        omega_ned = np.array(vehicle_odom.angular_velocity)

        #Body FLU
        v_ned_body = rot_ned.inv().apply(v_ned)
        omega_ned_body = rot_ned.inv().apply(omega_ned)

        #Body FLU
        v_body = R_ned_to_flu.apply(v_ned_body)
        omega_body = R_ned_to_flu.apply(omega_ned_body)
        
        # # Global FLU (rotated from NED)
        # v_flu_global = R_ned_to_flu.apply(v_ned)
        # omega_flu_global = R_ned_to_flu.apply(omega_ned)
        
        # # Rotate global FLU velocities into body frame (inverse of FLU orientation)
        # v_body = rot_ned.inv().apply(v_ned)
        # omega_body = rot_flu.inv().apply(omega_ned)
        
        # Nav2 message
        nav2_odom = Odometry()
        nav2_odom.header.stamp = self.get_clock().now().to_msg()
        nav2_odom.header.frame_id = "odom"
        nav2_odom.child_frame_id = "base_link"
        
        # Set position
        nav2_odom.pose.pose.position.x = pos_flu[0]
        nav2_odom.pose.pose.position.y = pos_flu[1]
        nav2_odom.pose.pose.position.z = pos_flu[2]
        
        # Set orientation
        nav2_odom.pose.pose.orientation.x = quat_flu[0]
        nav2_odom.pose.pose.orientation.y = quat_flu[1]
        nav2_odom.pose.pose.orientation.z = quat_flu[2]
        nav2_odom.pose.pose.orientation.w = quat_flu[3]

        # Set linear and angular velocities
        nav2_odom.twist.twist.linear.x = v_body[0]
        nav2_odom.twist.twist.linear.y = v_body[1]
        nav2_odom.twist.twist.linear.z = v_body[2]
        nav2_odom.twist.twist.angular.x = omega_body[0]
        nav2_odom.twist.twist.angular.y = omega_body[1]
        nav2_odom.twist.twist.angular.z = omega_body[2]

        # self.get_logger().info("linear: {}, angular: {}".format(
        #     v_body,
        #     omega_body
        # ))

        return nav2_odom

    ####################################################################################
    ####################################################################################
    # PX4 Mode Control commands
    ####################################################################################
    ####################################################################################

    def _px4_send_arm_cmd(self) -> bool:
        """Send the arming command to the px4

        Returns:
            bool: Command successfully sent
        """
        self.deactivate_process_vel_cmds_timer()
        self.deactivate_command_timer()

        if self.vehicle_status_latest.pre_flight_checks_pass:
            self._px4_send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info("Sent arming command")
            return True
        else:
            self.get_logger().info("Did not ARM because pre flight checks failed")
            return False

    def _px4_send_disarm_cmd(self):
        """Send the disarm command to the px4

        Returns:
            Bool: command sent successfully
        """
        self.deactivate_process_vel_cmds_timer()
        self.deactivate_command_timer()
        self._px4_send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Sent disarming command")

        return True

    def _px4_enable_offboard_control(self):
        self._px4_send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Sent offboard control command")

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
        self._px4_send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

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

    def _px4_send_vehicle_cmd(self, command, **params):
        """Publish a vehicle command."""
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
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    ####################################################################################
    ####################################################################################
    # ROS2 control commands
    ####################################################################################
    ####################################################################################

    def arm_status_callback(self, msg: Bool):

        if msg.data:
            self._px4_send_arm_cmd()
        else:
            self._px4_send_disarm_cmd()

    def takeoff_callback(self, msg: Bool):
        if not msg.data:
            return  # only takeoff:=true takes off
        if self.vehicle_status_latest.arming_state == VehicleStatus.ARMING_STATE_ARMED:
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
                    caller="velocity commander"
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
            caller="hovering"
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
