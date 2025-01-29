from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Spawn robot in Gazebo
        ExecuteProcess(
            cmd=['ign gazebo -r ackermann_steering.sdf'],
            output='screen'
        ),
        # Start the controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                'ackermann_drive_controller.yaml'
            ]
        ),
        # Load and start the Ackermann drive controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['ackermann_drive_controller'],
            output='screen'
        ),
    ])
