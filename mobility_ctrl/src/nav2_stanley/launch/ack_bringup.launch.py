#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = get_package_share_directory('nav2_stanley')

    # launch Gazebo simulation and spawn rover
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/ack_simulation.launch.py'))
    )

    # Launch Nav2 packages
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/ack_nav.launch.py'))
    )

    # Launch EKF node (currently only including IMU)
    # ack_localization_ukf uses UKF node instead (not in use)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/ack_localization.launch.py'))
    )


    return LaunchDescription(
        [
            navigation,
            localization,
            simulation
        ]
    )


