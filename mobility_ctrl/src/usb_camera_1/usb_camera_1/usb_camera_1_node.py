import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()

        # Explicitly use /dev/video2
        device_path = '/dev/video2'
        self.cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera at {device_path}')
            return

        self.get_logger().info(f'Successfully opened camera at {device_path}')
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish frame: {e}')
        else:
            self.get_logger().warn('Failed to read frame from camera')

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
