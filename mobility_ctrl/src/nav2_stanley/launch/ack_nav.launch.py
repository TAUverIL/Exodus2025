#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    pkg_share = get_package_share_directory('nav2_stanley')
    controller = LaunchConfiguration('controller')

    # Update navigation parameters (find yaml file) and map file path
    ack_params_file = os.path.join(pkg_share, 'config/ack_navigation.yaml')
    diff_params_file = os.path.join(pkg_share, 'config/diff_navigation.yaml')
    map_file = os.path.join(pkg_share, 'maps/ack_map.yaml')

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Start navigation
    ack_nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch/navigation_launch.py')),
        launch_arguments={'use_sim_time': 'True', 'params_file': ack_params_file}.items(),
        condition=IfCondition(PythonExpression(["'", controller, "' == 'ackermann'"]))
    )

    diff_nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch/navigation_launch.py')),
        launch_arguments={'use_sim_time': 'True', 'params_file': ack_params_file}.items(),
        condition=IfCondition(PythonExpression(["'", controller, "' == 'diff_drive'"]))
    )

    map_server_node = Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'yaml_filename': map_file}],
                arguments=['--ros-args', '--log-level', 'info'])
    
    map_server_lifecycle_node = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[{'use_sim_time': True},
                            {'autostart': True},
                            {'node_names': ['map_server']}])

    return LaunchDescription(
        [
            DeclareLaunchArgument(name='controller', default_value='ackermann', description='Choose controller to use'),
            ack_nav2_bringup_launch,
            diff_nav2_bringup_launch,
            map_server_node,
            map_server_lifecycle_node
        ]
    )


