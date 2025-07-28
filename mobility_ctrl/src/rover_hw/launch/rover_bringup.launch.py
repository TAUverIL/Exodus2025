import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_hw')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    
    declare_rviz_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=default_rviz_config_path,
        description='Full path to the RViz config file'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': False}],
    )
    
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/rover_description.launch.py'))
    )
    
    # Launch Nav2 packages
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/rover_nav.launch.py'))
    )

    # Launch EKF node (currently only including IMU)
    # ack_localization_ukf uses UKF node instead (not in use)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/rover_localization.launch.py'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'), 
        declare_rviz_arg, 
        rviz_node,
        description,
        navigation,
        localization
    ])