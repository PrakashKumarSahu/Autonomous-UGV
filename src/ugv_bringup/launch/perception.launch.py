import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time     = LaunchConfiguration('use_sim_time',     default='true')
    camera_type      = LaunchConfiguration('camera_type',      default='sim')
    enable_depth_viz = LaunchConfiguration('enable_depth_viz', default='false')

    # ══════════════════════════════════════════════════════════════════════════
    # DEPTH SOURCE — select ONE based on camera_type arg
    # All publish to the same canonical topics so the rest of the pipeline
    # is completely unaware of the hardware difference:
    #   /perception/depth/image_raw   (32FC1, metres, frame: camera_link_optical)
    #   /perception/depth/camera_info
    #   /perception/depth/colorized   (bgr8 TURBO — for RViz, no black image)
    # ══════════════════════════════════════════════════════════════════════════

    # ── camera_type:=sim ─────────────────────────────────────────────────────
    # Bridges Gazebo's real depth_camera sensor (metric ground-truth, 0.1–10 m).
    depth_relay_node = Node(
        package='ugv_perception',
        executable='depth_relay_node',
        name='depth_relay_node',
        parameters=[{
            'use_sim_time':        use_sim_time,
            'input_depth_topic':   '/camera/depth/image_raw',
            'input_info_topic':    '/camera/depth/camera_info',
            'output_depth_topic':  '/perception/depth/image_raw',
            'output_color_topic':  '/perception/depth/colorized',
            'output_info_topic':   '/perception/depth/camera_info',
            'output_frame_id':     'camera_link_optical',
            'fallback_info_topic': '/camera/camera_info',
            'colorize_max_depth':  10.0,
        }],
        output='screen',
        condition=IfCondition(EqualsSubstitution(camera_type, 'sim'))
    )

    # ── camera_type:=monocular ────────────────────────────────────────────────
    # Depth Anything V2 Metric Indoor — outputs metric depth in metres directly.
    # Model: depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf (~100MB,
    # cached in ~/.cache/huggingface/ after first run).
    # GPU: RTX 4050 runs at ~15 FPS. Frames are skipped when GPU is busy.
    depth_anything_node = Node(
        package='ugv_perception',
        executable='depth_node',
        name='depth_node',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'input_image_topic':  '/camera/image_raw',
            'camera_info_topic':  '/camera/camera_info',
            'output_depth_topic': '/perception/depth/image_raw',
            'output_color_topic': '/perception/depth/colorized',
            'depth_frame_id':     'camera_link_optical',
            'publish_pointcloud': False,
            'min_depth':          0.2,
            'max_depth':          10.0,
            'colorize_max_depth': 10.0,
            'model_id':           'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf',
        }],
        output='screen',
        condition=IfCondition(EqualsSubstitution(camera_type, 'monocular'))
    )

    # ── camera_type:=realsense ────────────────────────────────────────────────
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
    # Uses our custom node (RELIABLE QoS + proper camera_info topic) rather
    # than depth_image_proc/point_cloud_xyz_node (which had QoS mismatches).
    depth_to_pointcloud_node = Node(
        package='ugv_perception',
        executable='depth_to_pointcloud_node',
        name='depth_to_pointcloud_node',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'depth_topic':        '/perception/depth/image_raw',
            'camera_info_topic':  '/perception/depth/camera_info',
            'pointcloud_topic':   '/perception/depth/points',
            'step':               4,
        }],
        output='screen'
    )

    # ── YOLO11 Instance Segmentation ──────────────────────────────────────────
    # YOLO11n-seg: latest ultralytics segmentation model.
    # Model auto-downloads on first run (~7MB, cached by ultralytics).
    # Publishes hazard mask (binary) + visual overlay (bgr8) for RViz.
    yolo_seg_node = Node(
        package='ugv_perception',
        executable='yolo_seg_node',
        name='yolo_seg_node',
        parameters=[{
            'use_sim_time':         use_sim_time,
            'input_image_topic':    '/camera/image_raw',
            'output_mask_topic':    '/perception/hazard_mask',
            'output_overlay_topic': '/perception/yolo/overlay',
            'model_name':           'yolo11n-seg.pt',
            'confidence_threshold': 0.35,
        }],
        output='screen'
    )

    # ── Depth Anything V2 Visualization (enable_depth_viz:=true) ─────────────
    # Runs Depth Anything V2 Metric alongside the sim/hw depth source — purely
    # for visual comparison in RViz. NOT used by Nav2 or SLAM.
    # Disabled by default to save GPU memory. Enable with enable_depth_viz:=true.
    #
    # Publishes:
    #   /perception/depth_ai/image_raw  (32FC1, metric metres)
    #   /perception/depth_ai/colorized  (bgr8 TURBO — shown in RViz AIDepth panel)
    depth_viz_node = Node(
        package='ugv_perception',
        executable='depth_node',
        name='depth_viz_node',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'input_image_topic':  '/camera/image_raw',
            'camera_info_topic':  '/camera/camera_info',
            'output_depth_topic': '/perception/depth_ai/image_raw',
            'output_color_topic': '/perception/depth_ai/colorized',
            'depth_frame_id':     'camera_link_optical',
            'publish_pointcloud': False,
            'min_depth':          0.2,
            'max_depth':          10.0,
            'colorize_max_depth': 10.0,
            'model_id':           'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf',
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
                '"monocular" (Depth Anything V2 Metric) | '
                '"realsense" (Intel RealSense D435/D455) | '
                '"zed" (ZED 2/ZED X)'
            )
        ),
        DeclareLaunchArgument(
            'enable_depth_viz', default_value='false',
            description=(
                'Run Depth Anything V2 Metric as AI depth overlay for RViz comparison. '
                'Shows /perception/depth_ai/colorized alongside real depth. '
                'Costs ~2GB VRAM on RTX 4050. Default: false.'
            )
        ),
        # Depth sources (only ONE active at a time based on camera_type)
        depth_relay_node,
        depth_anything_node,
        realsense_relay_node,
        zed_relay_node,
        # Shared nodes (always active)
        depth_to_pointcloud_node,
        yolo_seg_node,
        # Optional visualization
        depth_viz_node,
    ])
