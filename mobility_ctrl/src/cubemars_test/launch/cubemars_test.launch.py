import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('cubemars_test')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('cubemars_test'),
                 'urdf', 'cubemars.xacro.urdf']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    config = os.path.join(pkg_share, 'config', 'cubemars_system.yaml')

    # bring up the hardware + controller_manager
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, config],
        output='screen'
    )

    # spawn the joint_state_broadcaster
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # spawn a simple effort controller
    effort = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller'],
        output='screen'
    )

    # spawn a simple position controller
    pos = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
        output='screen'
    )

    return LaunchDescription([control_node, jsb, pos])