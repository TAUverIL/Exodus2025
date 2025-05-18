#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CANTransceiverNode(Node):
    def __init__(self):
        super().__init__('can_transceiver_node')

        # Publisher to Jetson Orin
        self.publisher_to_jetson_orin = self.create_publisher(String, 'can_to_orin', 10)

        # Subscription from Jetson Orin
        self.subscription_from_orin = self.create_subscription(
            String,
            'orin_to_can',
            self.receive_from_orin,
            10
        )

        # Motor names
        self.motor_names = [
            'pivot_front_right', 'pivot_front_left',
            'pivot_rear_right', 'pivot_rear_left',
            'drive_front_right', 'drive_front_left',
            'drive_rear_right', 'drive_rear_left'
        ]

        # Dynamic publishers for all motors
        self.motor_publishers = {
            name: self.create_publisher(String, f'can_to_{name}', 10)
            for name in self.motor_names
        }

        # Subscriptions to all motor responses
        for name in self.motor_names:
            self.create_subscription(
                String,
                f'{name}_to_can',
                self.motor_feedback_callback_factory(name),
                10
            )

    def receive_from_orin(self, msg):
        self.get_logger().info(f'Received from Jetson Orin: {msg.data}')
        try:
            motor_name, command = msg.data.split(':', 1)
            motor_name = motor_name.strip()
            command = command.strip()

            if motor_name in self.motor_publishers:
                self.motor_publishers[motor_name].publish(String(data=command))
                self.get_logger().info(f'Forwarded to {motor_name}: {command}')
            else:
                self.get_logger().warn(f'Unknown motor name: {motor_name}')
        except ValueError:
            self.get_logger().error('Malformed message. Expected format: motor_name:command')

    def motor_feedback_callback_factory(self, motor_name):
        def callback(msg):
            log_msg = f'From {motor_name}: {msg.data}'
            self.get_logger().info(log_msg)
            self.publisher_to_jetson_orin.publish(String(data=log_msg))
        return callback

def main(args=None):
    rclpy.init(args=args)
    node = CANTransceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
