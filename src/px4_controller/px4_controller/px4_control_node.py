import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleOdometry
from std_msgs.msg import String,Bool
from geometry_msgs.msg import TwistStamped, PointStamped, TransformStamped
import time
import numpy as np
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class PX4ControlNode(Node):
    def __init__(self):
        super().__init__('px4_control_node')

        self.namespace = self.get_namespace()
        if self.namespace == '/':
            self.namespace = ''
        
        self.last_print_time = 0.0
        self.last_print_time = 0.0

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
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

        #publishers - ROS2 Nav2/odometry messages

        self.px4_to_nav_odom_publisher = self.create_publisher(
            Odometry, '/odom', qos_profile)
        
        
        #publishers - TF tree transformations
        
        #subscribers - PX4 messages
        self.vehicle_status_latest:VehicleStatus = VehicleStatus()
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

        #subscribers - ROS2 cmd/control commands (e.g.: keyop commands)
        self.armed_status_subscriber = self.create_subscription(
            msg_type=Bool,
            topic='{}/armed_status'.format(self.namespace),
            callback=self.arm_status_callback,
            qos_profile=qos_profile
        )

        self.takeoff_subscriber = self.create_subscription(
            msg_type=Bool,
            topic="{}/takeoff".format(self.namespace),
            callback=self.takeoff_callback,
            qos_profile=qos_profile
        )

        self.land_subscriber = self.create_subscription(
            msg_type=Bool,
            topic="{}/land".format(self.namespace),
            callback=self.land_callback,
            qos_profile=qos_profile
        )

        self.velocity_subscriber = self.create_subscription(
            msg_type=TwistStamped,
            topic="{}/velocity".format(self.namespace),
            callback=self.velocity_callback,
            qos_profile=qos_profile
        )

        self.rotate_subscriber = self.create_subscription(
            msg_type=Bool,
            topic="{}/rotate".format(self.namespace),
            callback=self.rotate_callback,
            qos_profile=qos_profile
        )

        self.default_altitude = 0.5

        self.current_position_ned = None
        self.current_q_ned = None

        self.offboard_setpoint_counter = 0

        #Timer
        self.timer = self.create_timer(0.2, self.publish_offboard_control)
        self.command_timer = None
        self.active_command = None
        self.rotation_step_timer = None

        # self.odometry_timer = self.create_timer(0.01, self.publish_nav_odometry)
        
        # TF broadcaster and timer for broadcasting transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.1, self.broadcast_tf)
        
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
    #PX4 Trajectory control commands
    ####################################################################################
    ####################################################################################

    def publish_trajectory_setpoint(
            self,
            position_ned:np.ndarray=np.array([np.nan,np.nan,np.nan]), 
            linear_ned:np.ndarray=np.array([0, 0, 0]),
            yaw_rad:float = np.nan,
            yaw_speed:float = 0.0):

        if self.vehicle_status_latest.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("Failed to send position command because not armed")
            return

        trajectory_setpoint = TrajectorySetpoint()
        trajectory_setpoint.timestamp = self.get_clock().now().nanoseconds // 1000
        trajectory_setpoint.position = position_ned
        trajectory_setpoint.velocity = linear_ned
        trajectory_setpoint.acceleration = np.array([0,0,0])
        trajectory_setpoint.yaw = yaw_rad
        trajectory_setpoint.yawspeed = yaw_speed

        self.active_command = trajectory_setpoint

        self.interrupt_command()

        self.command_timer = self.create_timer(0.1, self._publish_active_command)

    def _publish_active_command(self):
        self.active_command.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_setpoint_publisher.publish(self.active_command)


    ####################################################################################
    ####################################################################################
    #PX4 Subscriptions
    ####################################################################################
    ####################################################################################

    def vehicle_status_callback(self,msg:VehicleStatus):
        self.vehicle_status_latest = msg

    def vehicle_odometry_callback(self, msg:VehicleOdometry):
        # pos = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        self.current_position_ned = msg.position
        self.current_q_ned = msg.q
        self.current_yaw_speed = msg.angular_velocity[2]

        try:
            # Store the latest odometry message
            self.vehicle_odometry_latest = msg
            # Convert and publish immediately
            nav2_odometry = self.convert_px4_odometry_to_nav2(msg)
            self.px4_to_nav_odom_publisher.publish(nav2_odometry)

            current_time = time.time()
            if current_time - self.last_print_time > 1.0:
                # self.get_logger().info("Received and converted PX4 odometry")
                self.last_print_time = current_time
                
                
        except Exception as e:
            self.get_logger().error(f"Error in vehicle_odometry_callback: {str(e)}")


    #####################################################################################
    ####################################################################################
    # PX4 Odometry Conversion
    ####################################################################################

    def broadcast_tf(self):
        # Ensure we have a valid odometry message
        if not hasattr(self, "vehicle_odometry_latest"):
            return
        
        # Convert PX4 odometry to nav2 odometry
        nav2_odom = self.convert_px4_odometry_to_nav2(self.vehicle_odometry_latest)
        # test = self.convert_px4_odom_to_ros2(self.vehicle_odometry_latest)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"      # Parent frame
        t.child_frame_id = "base_link"    # Child frame
        t.transform.translation.x = nav2_odom.pose.pose.position.x
        t.transform.translation.y = nav2_odom.pose.pose.position.y
        t.transform.translation.z = nav2_odom.pose.pose.position.z
        t.transform.rotation = nav2_odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
         
   
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
        
        # Convert velocities through the same rotation chain
        v_ned = np.array(vehicle_odom.velocity)
        v_flu = R_ned_to_flu.apply(v_ned)
        # v_enu = R_flu_to_enu.apply(v_flu)
        
        omega_ned = np.array(vehicle_odom.angular_velocity)
        omega_flu = R_ned_to_flu.apply(omega_ned)
        # omega_enu = R_flu_to_enu.apply(omega_flu)
        
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
        nav2_odom.twist.twist.linear.x = v_flu[0]
        nav2_odom.twist.twist.linear.y = v_flu[1]
        nav2_odom.twist.twist.linear.z = v_flu[2]
        nav2_odom.twist.twist.angular.x = omega_flu[0]
        nav2_odom.twist.twist.angular.y = omega_flu[1]
        nav2_odom.twist.twist.angular.z = omega_flu[2]

        return nav2_odom

    ####################################################################################
    ####################################################################################
    #PX4 Mode Control commands
    ####################################################################################
    ####################################################################################


    def _px4_send_arm_cmd(self)->bool:
        """Send the arming command to the px4

        Returns:
            bool: Command successfully sent
        """

        self.interrupt_command()

        if self.vehicle_status_latest.pre_flight_checks_pass == True:
            self._px4_send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=1.0)
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
        self.interrupt_command()
        self._px4_send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Sent disarming command")
        
        return True
    
    def _px4_enable_offboard_control(self):
        self._px4_send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Sent offboard control command")
        
    
    def _px4_send_land_cmd(self):
        self.interrupt_command()
        self._px4_send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        #TODO: Add code to check that landing was successful
        self.get_logger().info("Sent land command")


    def _px4_send_takeoff_cmd(self):
        takeoff_position_ned = np.array(self.current_position_ned)
        takeoff_position_ned[2] = -self.default_altitude

        self.publish_trajectory_setpoint(
            position_ned=takeoff_position_ned,
            linear_ned=np.array([0,0,0])
        )

        # self._px4_send_vehicle_cmd(
        #     VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
        #     param7=altitude_m)
        # TODO: Add code to check that takeoff was successful
        
        self.get_logger().info("Sent takeoff command")

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
    #ROS2 control commands
    ####################################################################################
    ####################################################################################

    def arm_status_callback(self,msg:Bool):

        if msg.data == True:
            self._px4_send_arm_cmd()
        else:
            self._px4_send_disarm_cmd()

    def takeoff_callback(self,msg:Bool):
        if self.vehicle_status_latest.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self._px4_send_takeoff_cmd()
                #TODO: add behavior to wait until takeoff complete
        else:
            self.get_logger().info("Takeoff aborted because not armed")
    
    def land_callback(self,msg:Bool):
        if self.vehicle_status_latest.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self._px4_send_land_cmd()
                #TODO: add behavior to wait until landing complete


    def velocity_callback(self, msg:TwistStamped):

        try:
            R_flu_to_frd = Rotation.from_euler('x', 180, degrees=True)
            linear_flu = msg.twist.linear
            linear_frd = R_flu_to_frd.apply([linear_flu.x, linear_flu.y, linear_flu.z])

            q = self.current_q_ned
            r = Rotation.from_quat([q[1], q[2], q[3], q[0]])

            linear_ned = r.apply(linear_frd)

            angular = msg.twist.angular
            target_yaw_speed = angular.z

            target_position_ned = np.array([np.nan, np.nan, -self.default_altitude])
            target_linear_ned = np.array([linear_ned[0], linear_ned[1], 0])

            # Hover case
            if linear_ned[0] == 0 and linear_ned[1] == 0:
                target_position_ned = np.array(self.current_position_ned)
                target_position_ned[2] = -self.default_altitude
                target_linear_ned = np.array([0, 0, 0])

            self.publish_trajectory_setpoint(
                position_ned=target_position_ned,
                linear_ned=target_linear_ned,
                yaw_speed=target_yaw_speed
            )

            self.get_logger().info(f"Sent velocity command {linear_ned[0]}, {linear_ned[1]}")
            self.get_logger().info(f"Sent angular command {angular.z}")
            
        except Exception as e:
            self.get_logger().info(f"Failed to parse velocity callback: {e}")

    def rotate_callback(self, msg: Bool):

        q = self.current_q_ned
        r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        current_yaw = r.as_euler('zyx')[0]

        self.rotate_position = np.array(self.current_position_ned)
        self.rotate_position[2] = -self.default_altitude

        self.start_yaw = current_yaw
        if msg.data == True:
            self.target_yaw = (current_yaw - np.pi/2) % (2*np.pi) - np.pi
        else:
            self.target_yaw = (current_yaw + np.pi/2) % (2*np.pi) - np.pi

        self.yaw_step_size = np.deg2rad(4)
        self.current_yaw_step = 0

        if self.rotation_step_timer:
            self.rotation_step_timer.cancel()

        self.rotation_step_timer = self.create_timer(0.2, self.incremental_yaw_step)

    def incremental_yaw_step(self):
        yaw_diff = (self.target_yaw - self.start_yaw + np.pi) % (2*np.pi) - np.pi

        num_steps = int(np.abs(yaw_diff) / self.yaw_step_size)

        if self.current_yaw_step > num_steps:
            self.rotation_step_timer.cancel()
            return

        step_yaw = self.start_yaw + self.current_yaw_step * np.sign(yaw_diff) * self.yaw_step_size
        step_yaw = (step_yaw + np.pi) % (2*np.pi) - np.pi

        self.publish_trajectory_setpoint(
            position_ned=self.rotate_position,
            yaw_rad=step_yaw
        )
        self.current_yaw_step += 1

    def interrupt_command(self):
        if self.command_timer != None:
            self.command_timer.cancel()
            self.command_timer = None

        # if self.rotation_step_timer != None:
        #     self.rotation_step_timer.cancel()
        #     self.rotation_step_timer = None



def main(args=None):
    rclpy.init(args=args)
    px4_control_node = PX4ControlNode()
    try:
        rclpy.spin(px4_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__== '__main__':
    main()
