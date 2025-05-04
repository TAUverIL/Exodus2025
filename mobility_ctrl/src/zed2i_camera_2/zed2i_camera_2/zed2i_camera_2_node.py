#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

class Zed2iCamera2Node(Node):
    def __init__(self):
        super().__init__('zed2i_camera_2_node')

        self.publisher = self.create_publisher(Image, '/zed2i_camera_2/image_raw', 10)

        self.subscription = self.create_subscription(
            String,
            '/jetson_orin_nx_mcu_to_zed2i_camera_2',
            self.listener_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.publish_fake_image)

    def publish_fake_image(self):
        image = Image()
        image.header.stamp = self.get_clock().now().to_msg()
        image.height = 480
        image.width = 640
        image.encoding = 'rgb8'
        image.step = 640 * 3
        image.data = [0] * (640 * 480 * 3)
        self.publisher.publish(image)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = Zed2iCamera2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
