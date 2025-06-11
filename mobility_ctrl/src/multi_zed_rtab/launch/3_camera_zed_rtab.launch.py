from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml

def load_yaml_params(context, config_path):
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    zed_config = config.get('zed_config', {})
    launch_flags = config.get('launch_flags', {})
    rtabmap_params = config.get('rtabmap_params', {})

    from ament_index_python.packages import get_package_share_directory
    rtabmap_ros_dir = get_package_share_directory('rtabmap_ros')

    zed_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.expanduser('~'),
                'Exodus2025',
                'mobility_ctrl',
                'src',
                'zed-ros2-examples',
                'tutorials',
                'zed_multi_camera',
                'launch',
                'zed_multi_camera.launch.py'
            )
        ),
        launch_arguments=zed_config.items()
    )

    rgbd_sync1 = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        namespace='camera1',
        name='rgbd_sync1',
        remappings=[
            ('rgb/image', '/zed_multi/camera1/left/image_rect_color'),
            ('depth/image', '/zed_multi/camera1/depth/depth_registered'),
            ('rgb/camera_info', '/zed_multi/camera1/left/camera_info'),
        ],
        parameters=[{'approx_sync': True}],
    )

    rgbd_sync2 = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        namespace='camera2',
        name='rgbd_sync2',
        remappings=[
            ('rgb/image', '/zed_multi/camera2/left/image_rect_color'),
            ('depth/image', '/zed_multi/camera2/depth/depth_registered'),
            ('rgb/camera_info', '/zed_multi/camera2/left/camera_info'),
        ],
        parameters=[{'approx_sync': True}],
    )

    rgbd_sync3 = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        namespace='camera3',
        name='rgbd_sync3',
        remappings=[
            ('rgb/image', '/zed_multi/camera3/left/image_rect_color'),
            ('depth/image', '/zed_multi/camera3/depth/depth_registered'),
            ('rgb/camera_info', '/zed_multi/camera3/left/camera_info'),
        ],
        parameters=[{'approx_sync': True}],
    )

    rgbdx_sync = Node(
        package='rtabmap_sync',
        executable='rgbdx_sync',
        name='rgbdx_sync',
        namespace='rtabmap',
        remappings=[
            ('rgbd_image0', '/camera1/rgbd_image'),
            ('rgbd_image1', '/camera2/rgbd_image'),
            ('rgbd_image2', '/camera3/rgbd_image'),
            ('rgbd_images', '/rgbd_images'),
        ],
        parameters=[
            {'rgbd_cameras': 3},
            {'approx_sync': True},
            {'queue_size': 30}
        ]
    )

    odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='rtabmap',
        output='screen',
        remappings=[
            ('rgb/image', '/zed_multi/camera1/left/image_rect_color'),
            ('depth/image', '/zed_multi/camera1/depth/depth_registered'),
            ('rgb/camera_info', '/zed_multi/camera1/left/camera_info'),
            ('rgbd_image', '/camera1/rgbd_image'),
            ('rgbd_images', '/rgbd_images'),
            ('imu', '/zed_multi/camera1/imu/data'),
        ],
        parameters=[rtabmap_params]
    )
    
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        output='screen',
        arguments=['--delete_db_on_start'],
        remappings=[
            ('map', '/rtabmap/map'),
            ('rgb/image', '/zed_multi/camera1/left/image_rect_color'),
            ('depth/image', '/zed_multi/camera1/depth/depth_registered'),
            ('rgb/camera_info', '/zed_multi/camera1/left/camera_info'),
            ('rgbd_image', '/camera1/rgbd_image'),
            ('rgbd_images', '/rgbd_images'),
            ('left/image_rect', '/zed_multi/camera1/left/image_rect_color'),
            ('right/image_rect', '/zed_multi/camera2/left/image_rect_color'),  # 2nd camera
            # Add additional remaps for the third camera if needed by rtabmap
            ('left/camera_info', '/zed_multi/camera1/left/camera_info'),
            ('right/camera_info', '/zed_multi/camera2/left/camera_info'),
            # If you need to pass camera3 info, add more here as required by your node configuration
            ('scan', '/scan'),
            ('scan_cloud', '/scan_cloud'),
            ('user_data', '/user_data'),
            ('user_data_async', '/user_data_async'),
            ('gps/fix', '/gps/fix'),
            ('tag_detections', '/detections'),
            ('fiducial_transforms', '/fiducial_transforms'),
            ('odom', '/rtabmap/odom'),
            ('imu', '/zed_multi/camera1/imu/data'),
            ('goal_out', '/goal_pose'),
        ],
        parameters=[rtabmap_params]
    )

    rtabmapviz_node = Node(
        condition=IfCondition(str(launch_flags.get('rtabmapviz', 'true')).lower()),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        arguments=['-d', os.path.join(rtabmap_ros_dir, 'launch', 'config', 'rgbd_gui.ini')],
        remappings=[
            ('rgb/image', '/zed_multi/camera1/left/image_rect_color'),
            ('depth/image', '/zed_multi/camera1/depth/depth_registered'),
            ('rgb/camera_info', '/zed_multi/camera1/left/camera_info'),
            ('right/image_rect', '/zed_multi/camera2/left/image_rect_color'),  # 2nd camera
            # Optionally, remap camera3 if supported by your visualization
            ('left/camera_info', '/zed_multi/camera1/left/camera_info'),
            ('right/camera_info', '/zed_multi/camera2/left/camera_info'),
            ('scan', '/scan'),
            ('scan_cloud', '/scan_cloud'),
            ('odom', '/rtabmap/odom'),
        ],
        parameters=[rtabmap_params]
    )

    rviz_node = Node(
        condition=IfCondition(str(launch_flags.get('rviz', 'false')).lower()),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(rtabmap_ros_dir, 'launch', 'config', 'rgbd.rviz')],
    )

    return [
        zed_launcher,
        rgbd_sync1,
        rgbd_sync2,
        rgbd_sync3,
        rgbdx_sync,
        odometry_node,
        rtabmap_node,
        rtabmapviz_node,
        rviz_node
    ]

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'Exodus2025',
            'mobility_ctrl',
            'config',
            'multi_camera_config.yaml'
        ),
        description='Path to YAML config file.'
    )

    # Static transform: base_link -> camera1_camera_link
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera1_camera_link']
    )

    # Static transform: odom -> base_link
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    return LaunchDescription([
        config_file_arg,
        static_tf_base,
        static_tf_odom,
        OpaqueFunction(function=lambda context: load_yaml_params(
            context,
            LaunchConfiguration('config_file').perform(context)
        ))
    ])
