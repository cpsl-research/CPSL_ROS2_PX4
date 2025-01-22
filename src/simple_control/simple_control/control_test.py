import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
import time

class PX4ControlNode(Node):
    def __init__(self):
        super().__init__('px4_control_node')
        # Publishers
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # Timer to periodically send OffboardControlMode and TrajectorySetpoint
        self.timer = self.create_timer(0.1, self.publish_control)

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

    def arm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Sent arming command")
        self.armed = True

    def disarm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Sent disarming command")
        self.armed = False

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        vehicle_command = VehicleCommand()
        vehicle_command.timestamp = self.get_clock().now().nanoseconds // 1000  # Timestamp in microseconds
        vehicle_command.param1 = param1
        vehicle_command.param2 = param2
        vehicle_command.command = command
        vehicle_command.target_system = 1
        vehicle_command.target_component = 1
        vehicle_command.source_system = 1
        vehicle_command.source_component = 1
        vehicle_command.from_external = True
        self.vehicle_command_publisher.publish(vehicle_command)

def main(args=None):
    rclpy.init(args=args)

    px4_control_node = PX4ControlNode()

    # Give PX4 some time to initialize
    time.sleep(2)

    # Arm the drone
    px4_control_node.arm()
    time.sleep(2)

    # Start offboard mode
    px4_control_node.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
    px4_control_node.get_logger().info("Sent offboard mode command")
    time.sleep(5)  # Allow time for the drone to take off and stabilize

    try:
        rclpy.spin(px4_control_node)
    except KeyboardInterrupt:
        px4_control_node.disarm()
        px4_control_node.get_logger().info("Node stopped cleanly")
    finally:
        px4_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
