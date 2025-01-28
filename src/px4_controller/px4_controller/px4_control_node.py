import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
import time

class PX4ControlNode(Node):
    def __init__(self):
        super().__init__('px4_control_node')

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

        #publishers - ROS2 Nav2 messages

        #subscribers - PX4 messages

        #subscribers - ROS2 cmd/control commands (e.g.: keyop commands)

        #Timer
        self.timer = self.create_timer(0.01, self.publish_control)

        # State variables
        self.armed = False
        self.takeoff = False

    def publish_control(self):
        # Publish OffboardControlMode
        offboard_control_mode = OffboardControlMode()
        offboard_control_mode.timestamp = self.get_clock().now().nanoseconds // 1000  # Timestamp in microseconds
        offboard_control_mode.position = True
        offboard_control_mode.velocity = False
        offboard_control_mode.acceleration = False
        offboard_control_mode.attitude = False
        offboard_control_mode.body_rate = False
        self.offboard_control_mode_publisher.publish(offboard_control_mode)

        # Publish TrajectorySetpoint
        if self.armed:
            trajectory_setpoint = TrajectorySetpoint()
            trajectory_setpoint.timestamp = self.get_clock().now().nanoseconds // 1000  # Timestamp in microseconds
            trajectory_setpoint.position = [0.0, 0.0, -1.0]  # [x, y, z] (z is negative for altitude)
            trajectory_setpoint.yaw = 0.0
            self.trajectory_setpoint_publisher.publish(trajectory_setpoint)

    ####################################################################################
    ####################################################################################
    #Control commands
    ####################################################################################
    ####################################################################################

    def arm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Sent arming command")
        self.armed = True

    def disarm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Sent disarming command")
        self.armed = False
    
    def enable_offboard_control(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Sent offboard control command")
    
    def land(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Sent land command")

    def takeoff(self,altitude_m=1.0):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param7=altitude_m)

    def send_vehicle_command(self, command, **params):
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