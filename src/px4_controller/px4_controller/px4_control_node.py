import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleOdometry
from std_msgs.msg import String,Bool
from geometry_msgs.msg import TwistStamped, PointStamped
import time
import numpy as np
from scipy.spatial.transform import Rotation

class PX4ControlNode(Node):
    def __init__(self):
        super().__init__('px4_control_node')

        self.namespace = self.get_namespace()
        if self.namespace == '/':
            self.namespace = ''
        

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

        self.default_altitude = 1.0

        self.current_position_ned = None
        self.current_q = None

        self.offboard_setpoint_counter = 0

        #Timer
        self.timer = self.create_timer(0.2, self.publish_offboard_control)
        self.command_timer = None
        self.active_command = None

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

        self.command_timer = self.create_timer(1, self._publish_active_command);

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
        self.current_q = msg.q

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
            position_ned=takeoff_position_ned
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
            linear = msg.twist.linear
            angular = msg.twist.angular

            q = self.current_q
            r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
            yaw = r.as_euler('zyx')[0]

            vx = linear.x*np.cos(yaw) + linear.y*np.sin(yaw)
            vy = linear.x*np.sin(yaw) + linear.y*np.cos(yaw)

            target_position_ned = np.array([np.nan, np.nan, -self.default_altitude])
            target_linear_ned = np.array([vx, vy, 0])
            target_yaw_speed = -angular.z

            # Hover case
            if linear.x == 0 and linear.y == 0:
                target_position_ned = np.array(self.current_position_ned)
                target_position_ned[2] = -self.default_altitude
                target_linear_ned = np.array([0, 0, 0])

            self.publish_trajectory_setpoint(
                position_ned=target_position_ned,
                linear_ned=target_linear_ned,
                yaw_speed=target_yaw_speed
            )

            self.get_logger().info(f"Sent velocity command {linear.x}, {linear.y}, {linear.z}")
            self.get_logger().info(f"Sent angular command {angular.z}")
            
        except Exception as e:
            self.get_logger().info(f"Failed to parse velocity callback: {e}");

    def interrupt_command(self):
        if self.command_timer != None:
            self.command_timer.cancel()
        self.command_timer = None


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
