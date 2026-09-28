import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time  = LaunchConfiguration('use_sim_time',  default='true')
    camera_type   = LaunchConfiguration('camera_type',   default='sim')
    enable_depth_viz = LaunchConfiguration('enable_depth_viz', default='false')

    # ══════════════════════════════════════════════════════════════════════════
    # DEPTH SOURCE — select ONE based on camera_type arg
    # All publish to the same canonical topics so the rest of the pipeline
    # is completely unaware of the hardware difference:
    #   /perception/depth/image_raw   (32FC1, metres, frame: camera_link_optical)
    #   /perception/depth/camera_info
    # ══════════════════════════════════════════════════════════════════════════

    # ── camera_type:=sim ─────────────────────────────────────────────────────
    # Bridges Gazebo's real depth_camera sensor (metric ground-truth, 0.1-10 m).
    depth_relay_node = Node(
        package='ugv_perception',
        executable='depth_relay_node',
        name='depth_relay_node',
        parameters=[{
            'use_sim_time':        use_sim_time,
            'input_depth_topic':   '/camera/depth/image_raw',
            'input_info_topic':    '/camera/depth/camera_info',
            'output_depth_topic':  '/perception/depth/image_raw',
            'output_info_topic':   '/perception/depth/camera_info',
            'output_frame_id':     'camera_link_optical',
            'fallback_info_topic': '/camera/camera_info',   # RGB info fallback
        }],
        output='screen',
        condition=IfCondition(EqualsSubstitution(camera_type, 'sim'))
    )

    # ── camera_type:=monocular ────────────────────────────────────────────────
    # Depth Anything V3 monocular depth estimation from any RGB camera.
    # Use in production when only a single RGB camera is available.
    # Note: produces relative depth (scale ambiguity); adequate for obstacle
    # avoidance at UGV speeds. For metric accuracy, add floor-plane scale
    # calibration or fuse with IMU via RTAB-Map visual-inertial odometry.
    depth_anything_node = Node(
        package='ugv_perception',
        executable='depth_node',
        name='depth_node',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'input_image_topic':  '/camera/image_raw',
            'camera_info_topic':  '/camera/camera_info',
            'output_depth_topic': '/perception/depth/image_raw',
            'depth_frame_id':     'camera_link_optical',
            'publish_pointcloud': False,   # point_cloud_xyz_node handles this
            'min_depth':          0.2,
            'max_depth':          10.0,
        }],
        output='screen',
        condition=IfCondition(EqualsSubstitution(camera_type, 'monocular'))
    )

    # ── camera_type:=realsense ────────────────────────────────────────────────
    # Intel RealSense D435/D455/D457 — true metric depth, best for indoor.
    # Prerequisites: ros2 launch realsense2_camera rs_launch.py
    realsense_relay_node = Node(
        package='ugv_perception',
        executable='realsense_relay_node',
        name='realsense_relay_node',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'input_depth_topic':  '/camera/camera/depth/image_rect_raw',
            'input_info_topic':   '/camera/camera/depth/camera_info',
            'output_depth_topic': '/perception/depth/image_raw',
            'output_info_topic':  '/perception/depth/camera_info',
            'output_frame_id':    'camera_link_optical',
            'min_depth_m':        0.1,
            'max_depth_m':        10.0,
        }],
        output='screen',
        condition=IfCondition(EqualsSubstitution(camera_type, 'realsense'))
    )

    # ── camera_type:=zed ──────────────────────────────────────────────────────
    # Stereolabs ZED 2 / ZED X — outdoor, long range (up to 20 m).
    # Prerequisites: ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
    zed_relay_node = Node(
        package='ugv_perception',
        executable='zed_relay_node',
        name='zed_relay_node',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'input_depth_topic':  '/zed/zed_node/depth/depth_registered',
            'input_info_topic':   '/zed/zed_node/depth/camera_info',
            'output_depth_topic': '/perception/depth/image_raw',
            'output_info_topic':  '/perception/depth/camera_info',
            'output_frame_id':    'camera_link_optical',
        }],
        output='screen',
        condition=IfCondition(EqualsSubstitution(camera_type, 'zed'))
    )

    # ══════════════════════════════════════════════════════════════════════════
    # SHARED NODES — always run regardless of camera_type
    # ══════════════════════════════════════════════════════════════════════════

    # ── Depth Image → 3D PointCloud ───────────────────────────────────────────
    # Converts /perception/depth/image_raw + /perception/depth/camera_info
    # to /perception/depth/points (PointCloud2) used by Nav2 ObstacleLayer.
    depth_image_proc_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='point_cloud_xyz_node',
        output='screen',
        remappings=[
            ('image_rect',  '/perception/depth/image_raw'),
            ('camera_info', '/perception/depth/camera_info'),
            ('points',      '/perception/depth/points'),
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── YOLO Instance Segmentation ────────────────────────────────────────────
    # YOLOv8/YOLO11 runs on /camera/image_raw (RGB from Gazebo or real camera).
    # Publishes hazard mask fused into terrain analysis + visual overlay for RViz.
    yolo_seg_node = Node(
        package='ugv_perception',
        executable='yolo_seg_node',
        name='yolo_seg_node',
        parameters=[{
            'use_sim_time':        use_sim_time,
            'input_image_topic':   '/camera/image_raw',
            'output_mask_topic':   '/perception/hazard_mask',
            'model_name':          'yolov8n-seg.pt',
            'confidence_threshold': 0.35,
        }],
        output='screen'
    )

    # ── Optional: Depth Anything Visualization (enable_depth_viz:=true) ───────
    # Runs Depth Anything V3 purely for RViz visualization of AI depth estimate.
    # Does NOT feed into navigation. Useful to compare AI depth vs real depth.
    # Disabled by default to save GPU (enable with enable_depth_viz:=true).
    depth_viz_node = Node(
        package='ugv_perception',
        executable='depth_node',
        name='depth_viz_node',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'input_image_topic':  '/camera/image_raw',
            'camera_info_topic':  '/camera/camera_info',
            'output_depth_topic': '/perception/depth_ai/image_raw',   # separate topic
            'depth_frame_id':     'camera_link_optical',
            'publish_pointcloud': False,
            'min_depth':          0.2,
            'max_depth':          10.0,
        }],
        output='screen',
        condition=IfCondition(enable_depth_viz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'camera_type', default_value='sim',
            description=(
                'Depth source: '
                '"sim" (Gazebo depth sensor) | '
                '"monocular" (Depth Anything V3) | '
                '"realsense" (Intel RealSense D435/D455) | '
                '"zed" (ZED 2/ZED X)'
            )
        ),
        DeclareLaunchArgument(
            'enable_depth_viz', default_value='false',
            description='Run Depth Anything V3 as visual overlay (RViz only, not used by Nav2)'
        ),
        # Depth sources (only ONE active at a time based on camera_type)
        depth_relay_node,
        depth_anything_node,
        realsense_relay_node,
        zed_relay_node,
        # Shared nodes (always active)
        depth_image_proc_node,
        yolo_seg_node,
        depth_viz_node,
    ])
