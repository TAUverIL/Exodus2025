from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def load_yaml_params(context, config_path):
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    zed_config = config.get('zed_config', {})
    launch_flags = config.get('launch_flags', {})
    rtabmap_params = config.get('rtabmap_params', {})

    rtabmap_ros_dir = get_package_share_directory('rtabmap_ros')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

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
        parameters=[{'approx_sync': True},{'queue_size': 30}, {'use_sim_time':use_sim_time}]
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
        parameters=[{'approx_sync': True},{'queue_size': 30}, {'use_sim_time':use_sim_time}]
    )

    rgbdx_sync = Node(
        package='rtabmap_sync',
        executable='rgbdx_sync',
        name='rgbdx_sync',
        namespace='rtabmap',
        remappings=[
            ('rgbd_image0', '/camera1/rgbd_image'),
            ('rgbd_image1', '/camera2/rgbd_image'),
            ('rgbd_images', '/rgbd_images'),
        ],
        parameters=[
            {'rgbd_cameras': 2},
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
        parameters=[rtabmap_params, {'use_sim_time':use_sim_time}]
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
        ('right/image_rect', '/zed_multi/camera2/left/image_rect_color'),  # Assumed right = second camera left image
        ('left/camera_info', '/zed_multi/camera1/left/camera_info'),
        ('right/camera_info', '/zed_multi/camera2/left/camera_info'),      # Assumed right = second camera left camera_info
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
    parameters=[rtabmap_params,{'use_sim_time':use_sim_time}]
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
        ('rgbd_image', '/camera1/rgbd_image'),
        ('rgbd_images', '/rgbd_images'),
        ('left/image_rect', '/zed_multi/camera1/left/image_rect_color'),
        ('right/image_rect', '/zed_multi/camera2/left/image_rect_color'),  # Assumed as above
        ('left/camera_info', '/zed_multi/camera1/left/camera_info'),
        ('right/camera_info', '/zed_multi/camera2/left/camera_info'),      # Assumed as above
        ('scan', '/scan'),
        ('scan_cloud', '/scan_cloud'),
        ('odom', '/rtabmap/odom'),
    ],
    parameters=[rtabmap_params,{'use_sim_time':use_sim_time}]
)

    # rviz_node = Node(
    #     condition=IfCondition(str(launch_flags.get('rviz', 'false')).lower()),
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz',
    #     arguments=['-d', os.path.join(rtabmap_ros_dir, 'launch', 'config', 'rgbd.rviz')],
    # )

    return [
        zed_launcher,
        rgbd_sync1,
        rgbd_sync2,
        rgbdx_sync,
        odometry_node,
        rtabmap_node,
        rtabmapviz_node
        # rviz_node
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_hw')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'zed_config.yaml'),
        description='Path to YAML config file.'
    )
    
    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=lambda context: load_yaml_params(
            context,
            LaunchConfiguration('config_file').perform(context)
        ))
    ])
