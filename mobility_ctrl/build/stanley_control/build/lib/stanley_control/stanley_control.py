import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math


class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller_node')

        # Publishers
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, 
            '/ackermann_steering_controller/ackermann_cmd', 
            10
        )
        # Add visualization publisher
        self.viz_publisher = self.create_publisher(
            MarkerArray,
            'path_visualization',
            10
        )
        self.debug_publisher = self.create_publisher(
            Marker,
            'debug_vectors',
            10
        )

        # Subscribers
        self.subscriber = self.create_subscription(
            Odometry,
            '/ackermann_steering_controller/odometry',
            self.odom_callback,
            10
        )
        
        # Figure-8 path with closer points
        self.waypoints = np.array([
            [0.0, 0.0],
            [1.0, 1.0],
            [2.0, 2.0],
            [3.0, 3.0],
            [4.0, 4.0],
            [5.0, 5.0],
            [8.0, 8.0],
            [11.0, 11.0],
            [0.0, 0.0],
        ])

        #self.waypoints = np.array([
            #[0.0, 0.0],
            #[1.0, 1.0],
            #[2.0, 0.0],
            #[3.0, -1.0],
            #[4.0, 0.0],
            #[3.0, 1.0],
            #[2.0, 0.0],
            #[1.0, -1.0],
            #[0.0, 0.0],
        #])
        
        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('k', 0.5),
                ('k_soft', 2.0),
                ('max_steering', 0.7),
                ('max_speed', 0.3),
                ('goal_threshold', 0.2),
                ('wheelbase', 1.7),
                ('update_rate', 50.0)
            ]
        )
        
        # Get parameters
        self.k = self.get_parameter('k').value
        self.k_soft = self.get_parameter('k_soft').value
        self.max_steering = self.get_parameter('max_steering').value
        self.max_speed = self.get_parameter('max_speed').value
        self.goal_threshold = self.get_parameter('goal_threshold').value
        self.wheelbase = self.get_parameter('wheelbase').value
        update_rate = self.get_parameter('update_rate').value
        
        # State variables
        self.current_pose = None
        self.current_waypoint_idx = 0
        self.last_steering = 0.0
        
        # Create control loop timer
        self.timer = self.create_timer(1.0/update_rate, self.control_loop)
        
        # Publish initial path visualization
        self.publish_path_markers()
        
        self.get_logger().info('Stanley controller initialized with parameters:')
        self.get_logger().info(f'k: {self.k}, k_soft: {self.k_soft}')
        self.get_logger().info(f'max_steering: {self.max_steering}, max_speed: {self.max_speed}')
        self.get_logger().info(f'goal_threshold: {self.goal_threshold}')

    def publish_path_markers(self):
        marker_array = MarkerArray()
        
        # Path line strip
        line_strip = Marker()
        line_strip.header.frame_id = "odom"
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.scale.x = 0.1  # line width
        line_strip.color.a = 1.0
        line_strip.color.r = 0.0
        line_strip.color.g = 1.0
        line_strip.color.b = 0.0
        line_strip.pose.orientation.w = 1.0
        
        # Add points to line strip
        for point in self.waypoints:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            line_strip.points.append(p)
        
        marker_array.markers.append(line_strip)
        
        # Waypoint spheres
        for i, point in enumerate(self.waypoints):
            sphere = Marker()
            sphere.header.frame_id = "odom"
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.scale.x = 0.2
            sphere.scale.y = 0.2
            sphere.scale.z = 0.2
            sphere.color.a = 1.0
            sphere.color.r = 1.0
            sphere.color.g = 0.0
            sphere.color.b = 0.0
            sphere.pose.position.x = point[0]
            sphere.pose.position.y = point[1]
            sphere.pose.position.z = 0.0
            sphere.pose.orientation.w = 1.0
            sphere.id = i
            
            marker_array.markers.append(sphere)
        
        self.viz_publisher.publish(marker_array)

    def publish_debug_vectors(self, current_pos, heading_vector, cte_vector):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # shaft diameter
        marker.scale.y = 0.2  # head diameter
        marker.scale.z = 0.2  # head length
        
        # Heading vector (blue)
        marker.id = 0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        
        start = Point()
        start.x = current_pos[0]
        start.y = current_pos[1]
        
        end = Point()
        end.x = current_pos[0] + heading_vector[0]
        end.y = current_pos[1] + heading_vector[1]
        
        marker.points = [start, end]
        self.debug_publisher.publish(marker)
        
        # Cross-track error vector (red)
        marker.id = 1
        marker.color.r = 1.0
        marker.color.b = 0.0
        
        end.x = current_pos[0] + cte_vector[0]
        end.y = current_pos[1] + cte_vector[1]
        
        marker.points = [start, end]
        self.debug_publisher.publish(marker)

    def get_vehicle_yaw(self, pose):
        """Extract yaw angle from quaternion."""
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w
        
        # Calculate yaw (rotation around z-axis)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return yaw

    def calculate_path_heading(self, current_pos, target_pos):
        """Calculate desired heading angle to target."""
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        return np.arctan2(dy, dx)

    def calculate_cross_track_error(self, current_pos, target_pos, path_heading):
        """Calculate cross track error (signed distance from path)."""
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        
        # Calculate cross track error
        ct_error = (-dx * np.sin(path_heading) + dy * np.cos(path_heading))
        
        return ct_error

    def control_loop(self):
        if self.current_pose is None or self.current_waypoint_idx >= len(self.waypoints):
            return

        # Get current state
        current_pos = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y
        ])
        current_yaw = self.get_vehicle_yaw(self.current_pose)
        target_pos = self.waypoints[self.current_waypoint_idx]

        # Calculate distance to target
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        distance_to_target = np.sqrt(dx*dx + dy*dy)

        # Check if we've reached the current waypoint
        if distance_to_target < self.goal_threshold:
            self.current_waypoint_idx += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx-1}')
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info('Path completed!')
                return
            return

        # Calculate path heading and cross track error
        path_heading = self.calculate_path_heading(current_pos, target_pos)
        cross_track_error = self.calculate_cross_track_error(
            current_pos, target_pos, path_heading
        )

        # Calculate heading error
        heading_error = path_heading - current_yaw
        # Normalize to [-pi, pi]
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        # Stanley steering control law
        velocity = min(self.max_speed, distance_to_target)
        if velocity < 0.1:  # Minimum velocity threshold
            velocity = 0.1

        # Calculate steering angle
        stanley_term = np.arctan2(
            self.k * cross_track_error,
            self.k_soft + velocity
        )
        
        # Combine heading error and Stanley term
        steering_angle = heading_error + stanley_term

        # Add steering limit and smoothing
        steering_angle = np.clip(steering_angle, -self.max_steering, self.max_steering)
        
        # Smooth steering
        steering_angle = 0.7 * steering_angle + 0.3 * self.last_steering
        self.last_steering = steering_angle

        # Create and publish control command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(velocity)
        self.drive_publisher.publish(drive_msg)

        # Publish debug visualization
        heading_vector = [np.cos(current_yaw), np.sin(current_yaw)]
        cte_vector = np.array([-np.sin(path_heading), np.cos(path_heading)]) * cross_track_error
        self.publish_debug_vectors(current_pos, heading_vector, cte_vector)

        # Log detailed status
        self.get_logger().info(
            f"Pos: ({current_pos[0]:.2f}, {current_pos[1]:.2f}) | "
            f"Target: ({target_pos[0]:.1f}, {target_pos[1]:.1f}) | "
            f"Distance: {distance_to_target:.2f}m | "
            f"CTE: {cross_track_error:.2f}m | "
            f"Heading error: {np.degrees(heading_error):.1f}° | "
            f"Stanley term: {np.degrees(stanley_term):.1f}° | "
            f"Steering: {np.degrees(steering_angle):.1f}° | "
            f"Speed: {velocity:.2f}m/s"
        )

    def odom_callback(self, msg: Odometry):
        """Updates the rover's current position from odometry."""
        self.current_pose = msg.pose.pose

def main(args=None):
    rclpy.init(args=args)
    controller = StanleyController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()