#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zed2i_camera_1',
            executable='zed2i_camera_1_node',
            name='zed2i_camera_1_node',
            output='screen'
        ),
        Node(
            package='zed2i_camera_2',
            executable='zed2i_camera_2_node',
            name='zed2i_camera_2_node',
            output='screen'
        ),
        Node(
            package='zed2i_camera_3',
            executable='zed2i_camera_3_node',
            name='zed2i_camera_3_node',
            output='screen'
        ),
        Node(
            package='jetson_orin_nx_mcu',
            executable='jetson_orin_nx_mcu_node',
            name='jetson_orin_nx_mcu_node',
            output='screen'
        ),
        Node(
            package='fl_switch',
            executable='fl_switch_node',
            name='fl_switch_node',
            output='screen'
        ),
        Node(
            package='jetson_arm_control',
            executable='jetson_arm_control_node',
            name='jetson_arm_control_node',
            output='screen'
        ),
        Node(
            package='can_transceiver',
            executable='can_transceiver_node',
            name='can_transceiver_node',
            output='screen'
        ),

        # Pivot Motors
        Node(
            package='pivot_motor',
            executable='pivot_motor_node',
            name='pivot_front_right',
            output='screen',
            parameters=[{'motor_name': 'pivot_front_right'}]
        ),
        Node(
            package='pivot_motor',
            executable='pivot_motor_node',
            name='pivot_front_left',
            output='screen',
            parameters=[{'motor_name': 'pivot_front_left'}]
        ),
        Node(
            package='pivot_motor',
            executable='pivot_motor_node',
            name='pivot_rear_right',
            output='screen',
            parameters=[{'motor_name': 'pivot_rear_right'}]
        ),
        Node(
            package='pivot_motor',
            executable='pivot_motor_node',
            name='pivot_rear_left',
            output='screen',
            parameters=[{'motor_name': 'pivot_rear_left'}]
        ),

        # Drive Motors
        Node(
            package='drive_motor',
            executable='drive_motor_node',
            name='drive_front_right',
            output='screen',
            parameters=[{'motor_name': 'drive_front_right'}]
        ),
        Node(
            package='drive_motor',
            executable='drive_motor_node',
            name='drive_front_left',
            output='screen',
            parameters=[{'motor_name': 'drive_front_left'}]
        ),
        Node(
            package='drive_motor',
            executable='drive_motor_node',
            name='drive_rear_right',
            output='screen',
            parameters=[{'motor_name': 'drive_rear_right'}]
        ),
        Node(
            package='drive_motor',
            executable='drive_motor_node',
            name='drive_rear_left',
            output='screen',
            parameters=[{'motor_name': 'drive_rear_left'}]
        ),

        # Telecom Module
        Node(
            package='telecom_module',
            executable='telecom_module_node',
            name='telecom_module_node',
            output='screen'
        ),
        
        # Science Module
        Node(
            package='science_module',
            executable='science_module_node',
            name='science_module_node',
            output='screen'
        ),

        # Add More Nodes

    ])
