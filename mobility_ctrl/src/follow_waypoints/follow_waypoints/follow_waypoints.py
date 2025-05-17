#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import tf2_ros

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # tolerance for “close enough”
        self.goal_tolerance = 0.5  # meters

        # TF to check final pose
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # action client
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for /navigate_to_pose…')
        self._client.wait_for_server()
        self.get_logger().info('…connected!')

        # subscribe to incoming waypoint lists
        self._wp_sub = self.create_subscription(
            Path, 'waypoints', self._on_waypoints, 10)

        self._queue = []
        self._waiting_for_result = False
        self._last_goal = None

    def _on_waypoints(self, path_msg: Path):
        # load all poses into the queue and start
        self._queue = list(path_msg.poses)
        self.get_logger().info(f'Received {len(self._queue)} waypoints.')
        self._try_send_next()

    def _try_send_next(self):
        if self._waiting_for_result or not self._queue:
            return

        # pop next waypoint
        next_pose: PoseStamped = self._queue.pop(0)
        next_pose.header.stamp = self.get_clock().now().to_msg()
        self._last_goal = next_pose

        goal = NavigateToPose.Goal()
        goal.pose = next_pose

        self.get_logger().info(
            f'Sending waypoint → ({next_pose.pose.position.x:.2f}, '
            f'{next_pose.pose.position.y:.2f})'
        )
        self._waiting_for_result = True
        send_goal = self._client.send_goal_async(goal)
        send_goal.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Goal rejected :(')
            self._waiting_for_result = False
            # immediately try next
            self._try_send_next()
            return

        self.get_logger().info('Goal accepted; waiting for result…')
        result_fut = gh.get_result_async()
        result_fut.add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status

        # if succeeded, or within tolerance, treat as success
        success = (status == GoalStatus.STATUS_SUCCEEDED)
        if not success:
            try:
                t = self._tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
                dx = t.transform.translation.x - self._last_goal.pose.position.x
                dy = t.transform.translation.y - self._last_goal.pose.position.y
                dist = math.hypot(dx, dy)
                if dist <= self.goal_tolerance:
                    success = True
                    self.get_logger().info(
                        f'Within {self.goal_tolerance:.2f} m (error={dist:.2f}) → accepting.'
                    )
                else:
                    self.get_logger().warn(
                        f'Goal failed (status={status}), error={dist:.2f} m > {self.goal_tolerance:.2f}.'
                    )
            except Exception as e:
                self.get_logger().warn(
                    f'Goal failed (status={status}), TF lookup failed: {e}'
                )

        if success:
            self.get_logger().info('✅ Waypoint reached.')
        # else we already printed a warning

        # allow next goal
        self._waiting_for_result = False
        self._try_send_next()

def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
