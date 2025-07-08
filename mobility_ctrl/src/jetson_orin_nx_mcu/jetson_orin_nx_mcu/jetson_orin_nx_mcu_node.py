#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

class JetsonOrinNxMcuNode(Node):
    def __init__(self):
        super().__init__('jetson_orin_nx_mcu_node')

        # Publishers
        self.pub_camera_cmds = {
            1: self.create_publisher(String, 'jetson_orin_nx_mcu_to_zed2i_camera_1', 10),
            2: self.create_publisher(String, 'jetson_orin_nx_mcu_to_zed2i_camera_2', 10),
            3: self.create_publisher(String, 'jetson_orin_nx_mcu_to_zed2i_camera_3', 10)
        }
        self.pub_to_can = self.create_publisher(String, 'orin_to_can', 10)
        self.pub_to_switch = self.create_publisher(String, '/fl_switch/port1_in', 10)
        self.pub_to_telecom = self.create_publisher(String, '/orin_to_telecom', 10)

        # Subscribers: image topics
        self.sub_camera_images = [
            self.create_subscription(Image, '/zed2i_camera_1/image_raw', self.camera_callback_factory(1), 10),
            self.create_subscription(Image, '/zed2i_camera_2/image_raw', self.camera_callback_factory(2), 10),
            self.create_subscription(Image, '/zed2i_camera_3/image_raw', self.camera_callback_factory(3), 10)
        ]

        # Subscribers: CAN, switch, telecom, science
        self.create_subscription(String, 'can_to_orin', self.generic_listener_callback, 10)
        self.create_subscription(String, '/fl_switch/port1_out', self.generic_listener_callback, 10)
        self.create_subscription(String, '/telecom_to_orin', self.telecom_callback, 10)
        self.create_subscription(String, '/science_module_data', self.science_data_callback, 10)

        # Timer to simulate outbound messages
        self.timer = self.create_timer(1.0, self.publish_message)

    def camera_callback_factory(self, cam_id):
        def callback(msg):
            self.get_logger().info(f'Received image from ZED 2i Camera {cam_id}')
        return callback

    def generic_listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

    def telecom_callback(self, msg):
        self.get_logger().info(f'Received remote command via Telecom: {msg.data}')
        ack = String()
        ack.data = f'ACK: Executed remote command \"{msg.data}\"'
        self.pub_to_telecom.publish(ack)

    def science_data_callback(self, msg):
        self.get_logger().info(f'Received science data: {msg.data}')

    def publish_message(self):
        message = String()
        message.data = 'Data from jetson_orin_nx_mcu_node'

        for cam_id, publisher in self.pub_camera_cmds.items():
            publisher.publish(message)
            self.get_logger().info(f'Sent to ZED 2i Camera {cam_id}')

        self.pub_to_can.publish(message)
        self.get_logger().info('Sent to CAN Transceiver')

        self.pub_to_switch.publish(message)
        self.get_logger().info('Sent to FL Switch')

def main(args=None):
    rclpy.init(args=args)
    node = JetsonOrinNxMcuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
