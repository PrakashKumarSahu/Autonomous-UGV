import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup    = get_package_share_directory('ugv_bringup')
    ekf_config     = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    rtabmap_config = os.path.join(pkg_bringup, 'config', 'rtabmap.yaml')

    mode           = LaunchConfiguration('mode',           default='sim')
    use_sim_time   = LaunchConfiguration('use_sim_time',   default='true')
    enable_rtabmap = LaunchConfiguration('enable_rtabmap', default='true')

    # ── 1. EKF — hardware only ────────────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(EqualsSubstitution(mode, 'hw'))
    )

    # ── 2. RTAB-Map RGB-D Sync ────────────────────────────────────────────────
    # Synchronises RGB image + depth image + camera_info into a single
    # rtabmap_msgs/RGBDImage message consumed by the SLAM node.
    #
    # CRITICAL QoS notes:
    #   qos=1 → SENSOR_DATA (BestEffort, Volatile) for image/depth topics.
    #            Must match Gazebo bridge which publishes BestEffort.
    #   qos_camera_info=1 → SENSOR_DATA for camera_info.
    #            Gazebo bridge camera_info is also BestEffort.
    #   approx_sync_max_interval=0.1 → 100 ms window (generous for 15 Hz cameras)
    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync':              True,
            'approx_sync_max_interval': 0.1,
            'use_sim_time':             use_sim_time,
            'queue_size':               30,
            'qos':                      1,   # SENSOR_DATA (BestEffort) — matches Gazebo bridge
            'qos_camera_info':          1,   # SENSOR_DATA (BestEffort) — matches Gazebo bridge
            'depth_scale':              1.0,
        }],
        remappings=[
            # rgbd_sync subscribes to these exact topic names:
            ('rgb/image',        '/camera/image_raw'),
            ('depth/image',      '/perception/depth/image_raw'),
            ('rgb/camera_info',  '/camera/camera_info'),
            ('rgbd_image',       '/rtabmap/rgbd_image'),
        ],
        condition=IfCondition(enable_rtabmap)
    )

    # ── 3. RTAB-Map SLAM Core ─────────────────────────────────────────────────
    # Builds a 2D occupancy map (/map) from the RGB-D stream.
    # Publishes map→odom TF for Nav2 global localization.
    #
    # Key parameters:
    #   frame_id = base_footprint (robot base frame — MUST match URDF)
    #   odom_frame_id = odom       (frame Gazebo bridge publishes odom→base_footprint into)
    #   subscribe_rgbd = True      (use combined RGBDImage, not separate rgb/depth)
    #   Mem/IncrementalMemory      (online SLAM, not localization-only)
    #
    # The 'odom' input (remapped to /odometry/filtered) provides the wheel
    # odometry that anchors RTAB-Map's local pose estimates between loop closures.
    #
    # NOTE: delete_db_on_start=True here ensures a fresh map each run.
    # For production reuse: set to False and remove '-d' argument.
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_config, {
            'use_sim_time':              use_sim_time,
            'frame_id':                  'base_footprint',   # MUST match URDF base frame
            'map_frame_id':              'map',
            'odom_frame_id':             'odom',
            'publish_tf':                True,               # publishes map→odom TF
            'subscribe_rgbd':            True,
            'subscribe_rgb':             False,
            'subscribe_depth':           False,
            'subscribe_scan':            False,
            'approx_sync':               True,
            'queue_size':                30,
            'delete_db_on_start':        True,
            'wait_for_transform':        1.0,
            'tf_tolerance':              0.5,
            # Publish /map immediately at startup — do NOT wait for robot motion.
            # These are rtabmap_ros ROS 2 wrapper params (not librtabmap RTAB params).
            # map_always_update=true  → republish map on every SLAM iteration (~1 Hz)
            # publish_null_when_empty → publish empty OccupancyGrid before first keyframe
            'map_always_update':         True,
            'publish_null_when_empty':   True,
        }],
        arguments=['-d'],  # delete DB on start (same as delete_db_on_start param)
        remappings=[
            ('rgbd_image', '/rtabmap/rgbd_image'),
            ('odom',       '/odometry/filtered'),  # wheel odometry from Gazebo DiffDrive
            # RTAB-Map publishes OccupancyGrid on 'grid_map' → remap to /map for Nav2
            ('grid_map',   '/map'),
        ],
        condition=IfCondition(enable_rtabmap)
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode',           default_value='sim',  description='sim or hw'),
        DeclareLaunchArgument('use_sim_time',   default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('enable_rtabmap', default_value='true', description='Enable RTAB-Map SLAM'),
        ekf_node,
        rgbd_sync_node,
        rtabmap_slam_node,
    ])
