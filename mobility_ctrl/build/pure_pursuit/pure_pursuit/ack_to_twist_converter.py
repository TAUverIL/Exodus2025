import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class AckermannToTwistConverter(Node):
    def __init__(self):
        super().__init__('ackermann_to_twist_converter')
        
        # Subscriber to the AckermannDriveStamped topic
        self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_steering_controller/ackermann_cmd',
            self.ackermann_callback,
            10
        )
        
        # Publisher to the Twist topic
        self.twist_pub = self.create_publisher(Twist, '/ackermann_steering_controller/reference_unstamped', 10)
    
    def ackermann_callback(self, msg: AckermannDriveStamped):
        # Create a new Twist message
        twist_msg = Twist()

        # Convert Ackermann message to Twist (basic example)
        # Assuming linear x is speed, angular z is steering angle (converted to angular velocity)
        twist_msg.linear.x = msg.drive.speed
        twist_msg.angular.z = msg.drive.steering_angle
        
        # Publish the Twist message
        self.twist_pub.publish(twist_msg)
        self.get_logger().info(f'Published Twist: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    converter = AckermannToTwistConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
