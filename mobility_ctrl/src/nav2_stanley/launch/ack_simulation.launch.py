from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    pkg_share = FindPackageShare(package='nav2_stanley').find('nav2_stanley')
    default_model_path = os.path.join(pkg_share, 'src', 'urdf', 'main_rover.xacro.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    # default_world_path = os.path.join(pkg_share, 'world', 'empty.sdf')
    default_world_path = os.path.join(pkg_share, 'world', 'world_trial.sdf')
    controller = LaunchConfiguration('controller')

    # Robot description with xacro command
    xacro_file = PathJoinSubstitution([
        FindPackageShare('nav2_stanley'), 'urdf', 'main_rover.xacro.urdf'
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file,
        ' ',
        'controller:=',
        controller
    ])

    # Robot State Publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time}
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'ack_rover', '-allow_renaming', 'true'],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'robot_description': Command([
                FindExecutable(name='xacro'),
                ' ',
                LaunchConfiguration('model'),
                ' ',
                'controller:=',
                controller
            ])}, 
            {'use_sim_time': use_sim_time}
        ],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_ackermann_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'ackermann_steering_controller'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'ackermann'"]))
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'diff_drive_controller'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'diff_drive'"]))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # IMU (IGN -> ROS2)
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            # GPS (IGN -> ROS2)
            '/gps/fix@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat',
            # CAMERA (IGN -> ROS)
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            # DEPTH_CAMERA (IGN -> ROS)
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            # LIDAR (IGN -> ROS)
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='controller', default_value='ackermann', description='Choose controller to use'),
        # ExecuteProcess(
        #     cmd=['echo', 'Using ACKERMANN controller'],
        #     condition=IfCondition(PythonExpression([controller, " == 'ackermann'"]))
        # ),
        # ExecuteProcess(
        #     cmd=['echo', 'Using DIFF DRIVE controller'],
        #     condition=IfCondition(PythonExpression([controller, " == 'diff_drive'"]))
        # ),
        bridge,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', f'-r -v 4 {default_world_path}')]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_ackermann_controller, load_diff_drive_controller],
            )
        ),
        node_robot_state_publisher,
        joint_state_publisher_node,
        gz_spawn_entity,
        rviz_node,
    ])