import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JetsonArmNode(Node):

    def __init__(self):
        super().__init__('jetson_arm_node')
        # Publishers
        self.publisher_jetson_arm_control_to_fl_switch = self.create_publisher(String,
         '/fl_switch/port2_in', 10)

        # Subscribers
        self.subscription_fl_switch_to_jetson_arm_control = self.create_subscription(String, 
        '/fl_switch/port2_out', self.arm_command_callback, 10)
        self.get_logger().info('Jetson ARM Node has been started.')

    def arm_command_callback(self, msg):
        self.get_logger().info(f'Received arm command: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = JetsonArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

