#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FLSwitchNode(Node):
    def __init__(self):
        super().__init__('fl_switch_node')

        # Subscriptions
        self.subscription_port1 = self.create_subscription(
            String,
            '/fl_switch/port1_in',
            self.callback_port1,
            10
        )
        self.subscription_port2 = self.create_subscription(
            String,
            '/fl_switch/port2_in',
            self.callback_port2,
            10
        )

        # Publishers
        self.publisher_port1 = self.create_publisher(String, '/fl_switch/port1_out', 10)
        self.publisher_port2 = self.create_publisher(String, '/fl_switch/port2_out', 10)

        # Timer for connection status
        self.status_timer = self.create_timer(5.0, self.print_port_status)

    def callback_port1(self, msg):
        self.get_logger().info(f'[Port 1] Received: "{msg.data}"')

        if self.publisher_port2.get_subscription_count() > 0:
            self.publisher_port2.publish(msg)
            self.get_logger().info('[Port 1] Forwarded to Port 2')
        else:
            self.get_logger().warn('[Port 1] No subscriber on Port 2, message dropped')

    def callback_port2(self, msg):
        self.get_logger().info(f'[Port 2] Received: "{msg.data}"')

        if self.publisher_port1.get_subscription_count() > 0:
            self.publisher_port1.publish(msg)
            self.get_logger().info('[Port 2] Forwarded to Port 1')
        else:
            self.get_logger().warn('[Port 2] No subscriber on Port 1, message dropped')

    def print_port_status(self):
        self.get_logger().info('--- FL Switch Port Status ---')
        self.get_logger().info(f'→ Port 1 Out: {self.publisher_port1.get_subscription_count()} subscribers')
        self.get_logger().info(f'→ Port 2 Out: {self.publisher_port2.get_subscription_count()} subscribers')

def main(args=None):
    rclpy.init(args=args)
    node = FLSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
