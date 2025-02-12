import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_ackermann_node')

        # Publish steering and velocity commands
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/ackermann_steering_controller/ackermann_cmd', 10)

        # Subscribe to odometry to track the current position
        self.subscriber = self.create_subscription(Odometry, '/ackermann_steering_controller/odometry', self.odom_callback, 10)
        
        # Define waypoints [x, y] in meters
        self.target_poses = [
            [2.0, 2.0],
            [4.0, 8.0],
            [6.0, 6.0],
            [6.0, -6.0],
        ]
        
        self.current_pose = None
        self.pose_idx = 0
        self.lookahead_distance = 1.0  # Distance ahead to track the target
        self.max_speed = 1.0  # Max linear velocity in m/s

        self.timer = self.create_timer(0.1, self.timer_callback)  # Update at 10Hz
    
    def odom_callback(self, msg: Odometry):
        """Updates the rover's current position from odometry."""
        self.current_pose = msg.pose.pose

    def timer_callback(self):
        """Computes the required Ackermann steering and velocity to reach the next waypoint."""
        if self.current_pose is None or self.pose_idx >= len(self.target_poses):
            return
        
        target = self.target_poses[self.pose_idx]
        car_x, car_y = self.current_pose.position.x, self.current_pose.position.y
        goal_x, goal_y = target[0], target[1]

        # Compute distance and heading to target
        dx = goal_x - car_x
        dy = goal_y - car_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.5:  # Consider waypoint reached
            self.pose_idx += 1
            return
        
        # Compute the target steering angle using Pure Pursuit
        lookahead_x = goal_x - car_x
        lookahead_y = goal_y - car_y

        # Compute curvature = 2*y / (L^2)
        L = self.lookahead_distance
        curvature = 2 * lookahead_y / (L ** 2)
        steering_angle = math.atan(curvature * L)

        # Send drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = min(self.max_speed, distance)  # Reduce speed near target
        #drive_msg.drive.acceleration = 1.0  # Set a positive acceleration
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)

        self.get_logger().info(f"Driving to: ({goal_x}, {goal_y}) | Steering: {steering_angle:.2f} rad | Speed: {drive_msg.drive.speed:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
