from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ZED Front Camera
        Node(
            package='zed_wrapper',
            executable='zed_wrapper_node',
            namespace='zed_multi/zed_front',
            name='zed_front',
            parameters=[{'camera_name': 'zed_front'}]
        ),

        # ZED Rear Camera
        Node(
            package='zed_wrapper',
            executable='zed_wrapper_node',
            namespace='zed_multi/zed_rear',
            name='zed_rear',
            parameters=[{'camera_name': 'zed_rear'}]
        ),

        # RTAB-Map for ZED Front (no GUI)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            namespace='zed_front_rtabmap',
            name='rtabmap_front',
            output='screen',
            parameters=[{
                'frame_id': 'zed_front_camera_link',
                'odom_topic': '/zed_multi/zed_front/odom',
                'visual_odometry': False,
                'rgb_topic': '/zed_multi/zed_front/rgb/image_rect_color',
                'depth_topic': '/zed_multi/zed_front/depth/depth_registered',
                'camera_info_topic': '/zed_multi/zed_front/rgb/camera_info',
                'wait_imu_to_init': True,
                'imu_topic': '/zed_multi/zed_front/imu/data',
                'approx_sync': True,
                'rgbd_sync': True,
                'approx_rgbd_sync': True,
                'queue_size': 30,
                'subscribe_rgbd': True,
                'subscribe_scan': False,
                'subscribe_scan_cloud': False
            }]
        ),

        # RTAB-Map for ZED Rear (no GUI)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            namespace='zed_rear_rtabmap',
            name='rtabmap_rear',
            output='screen',
            parameters=[{
                'frame_id': 'zed_rear_camera_link',
                'odom_topic': '/zed_multi/zed_rear/odom',
                'visual_odometry': False,
                'rgb_topic': '/zed_multi/zed_rear/rgb/image_rect_color',
                'depth_topic': '/zed_multi/zed_rear/depth/depth_registered',
                'camera_info_topic': '/zed_multi/zed_rear/rgb/camera_info',
                'wait_imu_to_init': True,
                'imu_topic': '/zed_multi/zed_rear/imu/data',
                'approx_sync': True,
                'rgbd_sync': True,
                'approx_rgbd_sync': True,
                'queue_size': 30,
                'subscribe_rgbd': True,
                'subscribe_scan': False,
                'subscribe_scan_cloud': False
            }]
        ),

        # Map Merge Node
        Node(
            package='rtabmap_util',
            executable='map_merge',
            name='map_merge',
            output='screen',
            parameters=[{
                'merge_map': True,
                'merge_odom': True,
                'frame_id': 'map'
            }],
            remappings=[
                ('map0', '/zed_front_rtabmap/map'),
                ('map1', '/zed_rear_rtabmap/map'),
                ('odom0', '/zed_front_rtabmap/odom'),
                ('odom1', '/zed_rear_rtabmap/odom')
            ]
        ),

        # RTAB-Map Visualization GUI
        Node(
            package='rtabmap_viz',
            executable='rtabmapviz',
            name='rtabmapviz',
            output='screen',
            parameters=[{
                'frame_id': 'map'
            }],
            remappings=[
                ('map', '/map'),
                ('odom', '/zed_front_rtabmap/odom'),
                ('rgb/image', '/zed_multi/zed_front/rgb/image_rect_color'),
                ('depth/image', '/zed_multi/zed_front/depth/depth_registered'),
                ('camera_info', '/zed_multi/zed_front/rgb/camera_info')
            ]
        )
    ])
    
    
