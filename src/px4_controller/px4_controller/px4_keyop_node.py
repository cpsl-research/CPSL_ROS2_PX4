import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import String,Bool
from geometry_msgs.msg import TwistStamped, PointStamped
from pynput import keyboard
import time
import numpy as np
from scipy.spatial.transform import Rotation

class PX4Keyop(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')

        self.LINEAR_VELOCITY = 0.2
        self.ANGULAR_VELOCITY = 0.25

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

        #publishers for control commands
        self.armed_status_publisher = self.create_publisher(
            msg_type=Bool,
            topic='{}/armed_status'.format(self.get_namespace()),
            qos_profile=qos_profile
        )

        self.takeoff_publisher = self.create_publisher(
            msg_type=Bool,
            topic='{}/takeoff'.format(self.get_namespace()),
            qos_profile=qos_profile
        )

        self.land_publisher = self.create_publisher(
            msg_type=Bool,
            topic='{}/land'.format(self.get_namespace()),
            qos_profile=qos_profile
        )

        self.velocity_publisher = self.create_publisher(
            msg_type=TwistStamped,
            topic='{}/cmd_vel'.format(self.get_namespace()),
            qos_profile=qos_profile
        )

        self.rotate_publisher = self.create_publisher(
            msg_type=Bool,
            topic='{}/rotate'.format(self.get_namespace()),
            qos_profile=qos_profile
        )

        # self.hover_publisher = self.create_publisher(
        #     msg_type=TwistStamped,
        #     topic='{}/hover'.format(self.get_namespace()),
        #     qos_profile=qos_profile
        # )

#         self.vehicle_odometry_subscriber = self.create_subscription(
#             msg_type=VehicleOdometry,
#             topic='/fmu/out/vehicle_odometry',
#             callback=self.vehicle_odometry_callback,
#             qos_profile=qos_profile
#         )

        #reacting to keyboard
        self.listener:keyboard.Listener = keyboard.Listener(
            on_press=self.on_key_press
        )
        self.listener.start()

    ####################################################################################
    ####################################################################################
    #Handling key presses
    ####################################################################################
    ####################################################################################

    def on_key_press(self, key):

        linear = [0, 0, 0]
        angular = [0, 0, 0]

        try:
            # Map specific key presses to messages
            if key.char == 'a':
                self.get_logger().info("ARMING")
                self.send_arm_cmd()
            elif key.char == 'd':
                self.get_logger().info("DISARMING")
                self.send_disarm_cmd()
            elif key.char == 't':
                self.get_logger().info("TAKING OFF")
                self.send_takeoff_cmd()
            elif key.char == 'l':
                self.send_land_cmd()
                self.get_logger().info("LANDING")
            elif key.char == 'r':
                # while angular[2] < self.ANGULAR_VELOCITY:
                #     angular[2] += 0.05
                #     time.sleep(0.5)
                #     self.send_velocity_cmd(linear, angular)
                angular[2] = self.ANGULAR_VELOCITY
                self.send_velocity_cmd(linear, angular)
            elif key.char == '[':
                self.send_rotate_cmd(False)
                self.get_logger().info("ROTATE 90 COUNTERCLOCKWISE")
            elif key.char == ']':
                self.send_rotate_cmd(True)
                self.get_logger().info("ROTATE 90 CLOCKWISE")

        except AttributeError:

            if key== keyboard.Key.space:
                self.send_velocity_cmd(linear, angular)
                self.get_logger().info("HOVER")

            if key == keyboard.Key.up:
                linear[0] = self.LINEAR_VELOCITY
                self.send_velocity_cmd(linear, angular)
                self.get_logger().info("GO FORWARD")

            if key == keyboard.Key.down:
                linear[0] = -self.LINEAR_VELOCITY
                self.send_velocity_cmd(linear, angular)
                self.get_logger().info("GO BACKWARD")

            if key == keyboard.Key.left:
                linear[1] = self.LINEAR_VELOCITY
                self.send_velocity_cmd(linear, angular)
                self.get_logger().info("GO LEFT")

            if key == keyboard.Key.right:
                linear[1] = -self.LINEAR_VELOCITY
                self.send_velocity_cmd(linear, angular)
                self.get_logger().info("GO RIGHT")

    ####################################################################################
    ####################################################################################
    #Publishing PX4 control commands
    ####################################################################################
    ####################################################################################

    def send_arm_cmd(self):
        """
        Send the command to arm the PX4
        """
        msg = Bool()
        msg.data = True
        self.armed_status_publisher.publish(msg)
    
    def send_disarm_cmd(self):
        """
        Send the command to disarm the PX4
        """
        msg = Bool()
        msg.data = False
        self.armed_status_publisher.publish(msg)

    def send_takeoff_cmd(self):
        """
        Send the command to takeoff
        """
        msg = Bool()
        msg.data = True
        self.takeoff_publisher.publish(msg)
    
    def send_land_cmd(self):
        """
        Send the command to land
        """
        msg = Bool()
        msg.data = True
        self.land_publisher.publish(msg)

    def send_velocity_cmd(self, linear, angular):
        msg = TwistStamped()
        msg.twist.linear.x = float(linear[0])
        msg.twist.linear.y = float(linear[1])
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(angular[2])

        self.velocity_publisher.publish(msg)


    def send_rotate_cmd(self, direction):
        msg = Bool()
        msg.data = direction
        self.rotate_publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = PX4Keyop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
