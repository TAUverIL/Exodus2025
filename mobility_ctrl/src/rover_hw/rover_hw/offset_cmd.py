import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class OffsetCmd(Node):
    def __init__(self):
        super.__init__('offset_cmd_node')
        self.declare_parameter('steering_offset_left', 0.0)
        self.declare_parameter('steering_offset_right', 0.0)
        self.steering_offset_left = self.get_parameter('steering_offset_left').value
        self.steering_offset_right = self.get_parameter('steering_offset_right').value
        
        self.sub = self.create_subscription(
            Twist, '/offset_cmd', self.cmd_callback, 10
        )
        
        self.pub = self.create_publisher(
            Twist, '/ackermann_steering_controller/reference_unstamped', 10
        )
        
        self.get_logger.info(f"Offset publisher started with offset {self.steering_offset:.3f} radians")
        
def cmd_callback(self, msg):
    out = Twist()
    out.linear = msg.linear
    out.angular = msg.angular
    out.angular.z += self.steering_offset
    self.pub.publish(out)
    
def main(args=None):
    rclpy.init(args=args)
    node = OffsetCmd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    