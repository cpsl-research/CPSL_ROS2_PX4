import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String,Bool
from pynput import keyboard

class PX4Keyop(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')

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

        #reacting to keyboard
        self.listener:keyboard.Listener = keyboard.Listener(on_press=self.on_key_press)
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
        except AttributeError:
            # Handle special keys (like arrow keys, etc.) if needed
            pass

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