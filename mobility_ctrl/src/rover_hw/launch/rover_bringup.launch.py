import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

import os

from launch_ros.actions import Node, ExecuteProcess
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_hw')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('rover_hw'),
                 'urdf', 'rover.xacro.urdf']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    controller_config = os.path.join(pkg_share, 'config', 'ackermann_drive_controller.yaml')

    # bring up the hardware + controller_manager
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen'
    )

    # spawn the joint_state_broadcaster
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawn Ackermann steering controller
    ackermann = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        remappings=[
        ('/ackermann_steering_controller/reference_unstamped', '/cmd_vel'),
        ('/ackermann_steering_controller/tf_odometry', '/tf'),
        ('/ackermann_steering_controller/odometry', '/odom'),
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([control_node, jsb, ackermann, rviz_node])