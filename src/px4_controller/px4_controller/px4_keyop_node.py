import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import String,Bool
from geometry_msgs.msg import TwistStamped, PointStamped
from pynput import keyboard
import numpy as np
from scipy.spatial.transform import Rotation

class PX4Keyop(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')

        self.LINEAR_VELOCITY = 0.2
        self.ANGULAR_VELOCITY = 0.4

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
            topic='{}/velocity'.format(self.get_namespace()),
            qos_profile=qos_profile
        )

        self.position_publisher = self.create_publisher(
            msg_type=PointStamped,
            topic='{}/position'.format(self.get_namespace()),
            qos_profile=qos_profile
        )

        self.vehicle_odometry_subscriber = self.create_subscription(
            msg_type=VehicleOdometry,
            topic='/fmu/out/vehicle_odometry',
            callback=self.vehicle_odometry_callback,
            qos_profile=qos_profile
        )

        self.position_ned = [0, 0, 0]
        self.velocity_mappings = { }

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
            elif key.char == 'o':
                self.get_logger().info("ORIGIN")
                self.send_position_cmd([0, 0, 0])
            elif key.char == 'h':
                self.get_logger().info("HOVER")
                self.send_position_cmd(self.position_ned)
        except AttributeError:
            # Handle special keys (like arrow keys, etc.) if needed
            if key in self.velocity_mappings:
                linear, angular = self.velocity_mappings[key]
                self.get_logger().info(f"Moving {key}")
                self.send_velocity_cmd(linear, angular)


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
        msg.twist.linear.z = float(linear[2])

        msg.twist.angular.x = float(angular[0])
        msg.twist.angular.y = float(angular[1])
        msg.twist.angular.z = float(angular[2])
        self.velocity_publisher.publish(msg)

    def send_position_cmd(self, position):
        msg = PointStamped()
        msg.point.x = float(position[0])
        msg.point.y = float(position[1])
        msg.point.z = float(position[2])
        self.position_publisher.publish(msg)

    def vehicle_odometry_callback(self, msg:VehicleOdometry):
        q = msg.q

        # self.position_ned = [msg.x, msg.y, msg.z]
        self.position_ned = [msg.position[0], msg.position[1], msg.position[2]]

        r = Rotation.from_quat([q[1], q[2], q[3], q[0]])  
        yaw = r.as_euler('zyx')[0]

        self.velocity_mappings = {
            keyboard.Key.space: ([0, 0, 0], [0, 0, 0]),
            keyboard.Key.up: ([self.LINEAR_VELOCITY*np.cos(yaw), self.LINEAR_VELOCITY*np.sin(yaw), 0], [0, 0, 0]),
            keyboard.Key.down: ([-self.LINEAR_VELOCITY*np.cos(yaw), -self.LINEAR_VELOCITY*np.sin(yaw), 0], [0, 0, 0]),
            keyboard.Key.left: ([0, 0, 0], [0, 0, self.ANGULAR_VELOCITY]),
            keyboard.Key.right: ([0, 0, 0], [0, 0, -self.ANGULAR_VELOCITY]),
        }


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
