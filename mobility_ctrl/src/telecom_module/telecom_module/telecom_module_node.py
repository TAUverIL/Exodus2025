#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TelecomModuleNode(Node):
    def __init__(self):
        super().__init__('telecom_module_node')

        # Simulated uplink: commands from ground station
        self.uplink_publisher = self.create_publisher(String, '/telecom_to_orin', 10)

        # Simulated downlink: responses from Jetson Orin
        self.downlink_subscription = self.create_subscription(
            String,
            '/orin_to_telecom',
            self.receive_from_orin,
            10
        )

        # Timer to simulate remote command being sent
        self.timer = self.create_timer(5.0, self.send_uplink_command)

    def send_uplink_command(self):
        msg = String()
        msg.data = 'RemoteCommand: Test'
        self.uplink_publisher.publish(msg)
        self.get_logger().info(f'Sent uplink: {msg.data}')

    def receive_from_orin(self, msg):
        self.get_logger().info(f'Received downlink: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TelecomModuleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

