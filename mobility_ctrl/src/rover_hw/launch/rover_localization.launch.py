#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_share = get_package_share_directory('rover_hw')
    
    # multi_zed_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_share, 'launch', 'rover_zed.launch.py')
    #     ),
    #     launch_arguments={'config_file': os.path.join(pkg_share, 'config', 'zed_config.yaml')}.items()
    # )
    
    multi_zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('multi_zed_rtab'), 'launch', 'multi_zed_rtab.launch.py')
        )
        # launch_arguments={'config_file': os.path.join(get_package_share_directory('multi_'), 'config', 'zed_config.yaml')}.items()
    )

    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Flag to enable use_sim_time'),
        multi_zed_launch,
        ekf_localization_node
    ])


