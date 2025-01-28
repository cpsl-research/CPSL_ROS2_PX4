import rclpy
from rclpy.node import Node
import time

from px4_controller.px4_control_node import PX4ControlNode

def main(args=None):
    px4_control_node = PX4ControlNode()

    # Give PX4 some time to initialize
    time.sleep(2)

    # Start offboard mode
    px4_control_node.enable_offboard_control()
    px4_control_node.get_logger().info("Sent offboard mode command")

    # Arm the drone
    px4_control_node.arm()
    time.sleep(2)

    #takeoff
    px4_control_node.takeoff(altitude_m=1.5)

    time.sleep(5)

    px4_control_node.land()
    time.sleep(5)  # Allow time for the drone to take off and stabilize

    try:
        rclpy.spin(px4_control_node)
    except KeyboardInterrupt:
        px4_control_node.disarm()
        px4_control_node.get_logger().info("Node stopped cleanly")
    finally:
        px4_control_node.destroy_node()
        rclpy.shutdown()

if __name__== '__main__':
    main()