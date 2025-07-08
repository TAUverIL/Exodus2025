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

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_hw')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')


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
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}]
        #parameters=[{'robot_description': robot_description}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

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
    
    # run command: ros2 topic pub /rear_steering_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
    # rear_steering = Node(
    # package='controller_manager',
    # executable='spawner',
    # arguments=['rear_steering_controller'],
    # output='screen'
    # )
    
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

    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        node_robot_state_publisher,
        control_node, 
        jsb, 
        ackermann, 
        declare_rviz_arg, 
        rviz_node
    ])