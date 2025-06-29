#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_share = get_package_share_directory('nav2_stanley')
    # Path to the multi_zed_rtab.launch.py file (adjust package and path as needed)
    multi_zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('multi_zed_rtab'),  # Replace with the correct package if needed
                'launch',
                'multi_zed_rtab.launch.py'
            )
        ),
        # Optionally pass launch arguments here
        # launch_arguments={'config_file': LaunchConfiguration('config_file')}.items()
    )

    # ekf_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_node',
    #     output='screen',
    #     parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='true', description='Flag to enable use_sim_time'),
        multi_zed_launch,  # This line adds your multi-zed launch!
        # ekf_localization_node,
    ])


