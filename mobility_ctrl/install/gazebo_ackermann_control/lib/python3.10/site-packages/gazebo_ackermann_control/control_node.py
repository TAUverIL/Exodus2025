import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class AckermannControlNode(Node):
    def __init__(self):
        super().__init__('ackermann_control_node')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/ackermann_steering_controller/reference_unstamped', 10
        ) 

        # Subscriber for odometry
        self.odometry_subscriber = self.create_subscription(
            Odometry, '/ackermann_steering_controller/odometry', self.odometry_callback, 10
        )

        # Parameters
        self.max_linear_speed = 1.0  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.square_limit = 5.0  # meters from the origin

        self.get_logger().info("Ackermann control node initialized!")

    def odometry_callback(self, msg: Odometry):
        # Get current position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Calculate corrections
        if abs(x) < self.square_limit or abs(y) < self.square_limit:
            self.correct_position(x, y)
        else:
            self.stop_rover()

    def correct_position(self, x, y):
        # Compute angle back toward origin
        angle_to_origin = math.atan2(-y, -x)

        # Create a velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = min(self.max_linear_speed, 0.5)  # Move forward
        cmd_vel.angular.z = angle_to_origin  # Rotate toward origin

        # Publish the velocity command
        self.cmd_vel_publisher.publish(cmd_vel)
        self.get_logger().info(f"Correcting position: x={x:.2f}, y={y:.2f}")

    def stop_rover(self):
        # Stop the rover when within bounds
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)
        self.get_logger().info("Rover is within bounds, stopping.")

def main(args=None):
    rclpy.init(args=args)
    node = AckermannControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
