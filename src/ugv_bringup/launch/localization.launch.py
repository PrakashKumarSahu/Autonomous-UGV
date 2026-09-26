import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('ugv_bringup')
    ekf_config = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    rtabmap_config = os.path.join(pkg_bringup, 'config', 'rtabmap.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_rtabmap = LaunchConfiguration('enable_rtabmap', default='true')

    # 1. robot_localization EKF State Estimator (Sensor Fusion)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered')]
    )

    # 2. RTAB-Map RGB-D Sync Node
    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.1,
            'use_sim_time': use_sim_time,
            'queue_size': 20
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('depth/image', '/perception/depth/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('rgbd_image', '/rtabmap/rgbd_image')
        ],
        condition=IfCondition(enable_rtabmap)
    )

    # 3. RTAB-Map Visual Odometry Node
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[rtabmap_config, {
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'rtabmap_odom',
            'publish_tf': False,  # EKF publishes the active odom -> base_footprint TF
            'subscribe_rgbd': True,
            'subscribe_rgb': False,
            'subscribe_depth': False,
            'approx_sync': True,
            'queue_size': 20
        }],
        remappings=[
            ('rgbd_image', '/rtabmap/rgbd_image'),
            ('odom', '/rtabmap/odom')
        ],
        condition=IfCondition(enable_rtabmap)
    )

    # 4. RTAB-Map SLAM Core Node
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_config, {
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'subscribe_rgbd': True,
            'subscribe_rgb': False,
            'subscribe_depth': False,
            'approx_sync': True,
            'queue_size': 20,
            'delete_db_on_start': True
        }],
        arguments=['-d'],
        remappings=[
            ('rgbd_image', '/rtabmap/rgbd_image'),
            ('odom', '/odometry/filtered'),
            ('grid_map', '/map')
        ],
        condition=IfCondition(enable_rtabmap)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('enable_rtabmap', default_value='true', description='Enable RTAB-Map SLAM'),
        ekf_node,
        rgbd_sync_node,
        rtabmap_odom_node,
        rtabmap_slam_node
    ])
