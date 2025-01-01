from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '/home/brittc/ros2_ws/src/model.sdf'],
            output='screen'
        ),
        # ROS 2 bridge for command topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/rover/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
            output='screen',
        ),
    ])
