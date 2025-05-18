#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class ScienceModuleNode(Node):
    def __init__(self):
        super().__init__('science_module_node')

        self.publisher = self.create_publisher(String, '/science_module_data', 10)
        self.timer = self.create_timer(2.0, self.publish_fake_data)

    def publish_fake_data(self):
        temperature = round(random.uniform(-20.0, 35.0), 1)
        radiation = round(random.uniform(0.01, 0.15), 3)
        humidity = round(random.uniform(10.0, 90.0), 1)

        data = f'Temperature: {temperature}°C, Radiation: {radiation} mSv, Humidity: {humidity}%'
        msg = String()
        msg.data = data
        self.publisher.publish(msg)
        self.get_logger().info(f'Published science data: {data}')

def main(args=None):
    rclpy.init(args=args)
    node = ScienceModuleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

