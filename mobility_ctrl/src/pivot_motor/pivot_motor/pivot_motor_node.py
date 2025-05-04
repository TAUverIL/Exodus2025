import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PivotMotorNode(Node):
    def __init__(self):
        super().__init__('pivot_motor_node')
        self.declare_parameter('motor_name', 'unnamed_pivot')
        motor_name = self.get_parameter('motor_name').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            String,
            f'can_to_{motor_name}',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            f'{motor_name}_to_can',
            10
        )

        self.motor_name = motor_name
        self.get_logger().info(f'{motor_name} Pivot motor initialized.')

    def listener_callback(self, msg):
        self.get_logger().info(f'{self.motor_name} Received: {msg.data}')
        response = String()
        response.data = f'ACK from {self.motor_name}'
        self.publisher.publish(response)

def main(args=None):
    rclpy.init(args=args)
    node = PivotMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

