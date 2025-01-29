import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint,VehicleStatus
from std_msgs.msg import String,Bool
import time
import numpy as np

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

        #Timer
        self.timer = self.create_timer(0.01, self.publish_control)
        
        #turn on offboard control
        self._px4_enable_offboard_control()

        return
    
    def publish_control(self):
        # Publish OffboardControlMode
        offboard_control_mode = OffboardControlMode()
        offboard_control_mode.timestamp = self.get_clock().now().nanoseconds // 1000  # Timestamp in microseconds
        offboard_control_mode.position = True
        offboard_control_mode.velocity = True
        offboard_control_mode.acceleration = False
        offboard_control_mode.attitude = False
        offboard_control_mode.body_rate = False
        self.offboard_control_mode_publisher.publish(offboard_control_mode)



    ####################################################################################
    ####################################################################################
    #PX4 Trajectory control commands
    ####################################################################################
    ####################################################################################

    def send_trajectory_position_command(self,position_ned:np.ndarray, yaw_rad:float = 0.0):
        """Send a trajectory setpoint message to send the UAV to a specific position


        Args:
            position_ned (np.ndarray): _description_
            yaw_rad (float, optional): _description_. Defaults to 0.0.
        """
        if self.vehicle_status_latest.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            trajectory_setpoint = TrajectorySetpoint()
            trajectory_setpoint.timestamp = self.get_clock().now().nanoseconds // 1000  # Timestamp in microseconds
            trajectory_setpoint.position = position_ned
            trajectory_setpoint.velocity = np.array([np.nan,np.nan,np.nan])
            trajectory_setpoint.acceleration = np.array([np.nan,np.nan,np.nan])
            trajectory_setpoint.yaw = yaw_rad
            trajectory_setpoint.yawspeed = np.nan
            self.trajectory_setpoint_publisher.publish(trajectory_setpoint)
        else:
            self.get_logger().info("Failed to send position command because not armed")

    ####################################################################################
    ####################################################################################
    #PX4 Subscriptions
    ####################################################################################
    ####################################################################################

    def vehicle_status_callback(self,msg:VehicleStatus):
        self.vehicle_status_latest = msg

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
        self._px4_send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Sent disarming command")
        
        return True
    
    def _px4_enable_offboard_control(self):
        self._px4_send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Sent offboard control command")
        
    
    def _px4_send_land_cmd(self):
        self._px4_send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        #TODO: Add code to check that landing was successful
        self.get_logger().info("Sent land command")


    def _px4_send_takeoff_cmd(self,altitude_m=1.0):
        self.send_trajectory_position_command(
            position_ned=np.array([0.0,0.0,-1.0])
        )
        # self._px4_send_vehicle_cmd(
        #     VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
        #     param7=altitude_m)
        #TODO: Add code to check that takeoff was successful
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

        if self.vehicle_status_latest.arming_state == \
            VehicleStatus.ARMING_STATE_ARMED:
                self._px4_send_takeoff_cmd()
                #TODO: add behavior to wait until takeoff complete
        else:
            self.get_logger().info("Takeoff aborted because not armed")
    
    def land_callback(self,msg:Bool):

        if self.vehicle_status_latest.arming_state == \
            VehicleStatus.ARMING_STATE_ARMED:
                self._px4_send_land_cmd()
                #TODO: add behavior to wait until landing complete


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