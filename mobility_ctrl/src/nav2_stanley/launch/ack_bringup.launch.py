#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = get_package_share_directory('nav2_stanley')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/ack_simulation.launch.py'))
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/ack_nav.launch.py'))
    )

    # visualization = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/visualization.launch.py'))
    # )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/ack_loc2.launch.py'))
    )


    return LaunchDescription(
        [
            simulation,
            navigation,
            # visualization,
            localization
        ]
    )


