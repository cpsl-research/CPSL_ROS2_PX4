import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import String,Bool
from geometry_msgs.msg import TwistStamped, PointStamped
from sensor_msgs.msg import Joy
from pynput import keyboard
import time
import numpy as np
from scipy.spatial.transform import Rotation

#reference for joy pkg: https://docs.ros.org/en/ros2_packages/jazzy/api/joy/index.html

class PX4Joy(Node):
    def __init__(self):
        super().__init__('px4_joy_node')

        self.max_linear_velocity = 0.2
        self.max_angular_velocity = 0.25

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #publishers for control commands
        self.armed_status:Bool = False
        self.armed_status_publisher = self.create_publisher(
            msg_type=Bool,
            topic='/armed_status',
            qos_profile=qos_profile
        )

        self.takeoff_publisher = self.create_publisher(
            msg_type=Bool,
            topic='/takeoff',
            qos_profile=qos_profile
        )

        self.land_publisher = self.create_publisher(
            msg_type=Bool,
            topic='/land',
            qos_profile=qos_profile
        )

        self.velocity_publisher = self.create_publisher(
            msg_type=TwistStamped,
            topic='/velocity',
            qos_profile=qos_profile
        )

        self.rotate_publisher = self.create_publisher(
            msg_type=Bool,
            topic='/rotate',
            qos_profile=qos_profile
        )

        #subscribers to joystick commands
        self.joy_subscriber = self.create_subscription(
            msg_type=Joy,
            topic='/joy',
            callback=self.joy_callback
        )


    ####################################################################################
    ####################################################################################
    #Handling key presses
    ####################################################################################
    ####################################################################################

    def joy_callback(self,msg:Joy):
        
        #handle buttons
        buttons = msg.buttons
        
        #takeoff command (trianle)
        if buttons[2] == 1:
            self.get_logger().info("TAKEOFF PRESSED")
            self.send_takeoff_cmd()
        
        #land command (cross)
        if buttons[0] == 1:
            self.get_logger().info("LANDING PRESSED")
            self.send_land_cmd()
        
        #arm command (right trigger)
        if buttons[7] == 1 and self.armed_status == False:
            self.get_logger().info("ARM PRESSED")
            self.send_arm_cmd()
            self.armed_status = True
        
        #disarm command (left trigger)
        if buttons[6] == 1:
            self.get_logger().info("DISARM PRESSED")
            self.send_disarm_cmd()
            self.armed_status = False
        
        #handle the control inputs
        linear = [0,0,0]
        angular = [0,0,0]

        cmds = msg.axes
        
        #x-velocity (up/down left joystick)
        linear[0] = self.max_linear_velocity * cmds[1]
        #y - velocity (left/right left joystick)
        linear[1] = self.max_linear_velocity * cmds[0] * -1

        #yaw rate
        angular[2] = self.max_angular_velocity * cmds[3] * -1


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
        
def main(args=None):
    rclpy.init(args=args)
    node = PX4Joy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
