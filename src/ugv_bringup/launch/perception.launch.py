"""
perception.launch.py — Perception stack (depth source + point cloud + YOLO26n-seg).

All node parameters live in ugv_perception/config/perception.yaml; this file only
decides WHICH nodes run. Override the config without rebuilding:
    perception_config:=/path/to/my_perception.yaml

Depth source (exactly one, chosen by camera_type):
    sim        → depth_relay_node      (Gazebo real depth_camera sensor)
    da_v3_trt  → da3_relay_node        (Depth Anything V3 TensorRT — requires TRT)
                 + depth_anything_v3   (C++ TRT node launched alongside relay)
    realsense  → realsense_relay_node  (Intel RealSense D435/D455)
    zed        → zed_relay_node        (Stereolabs ZED 2 / ZED X)

Always on:
    depth_to_pointcloud_node  /perception/depth/image_raw → /perception/depth/points
    yolo_seg_node             /camera/image_raw → /perception/hazard_mask + overlay

For da_v3_trt: see third_party/ros2-depth-anything-v3-trt and
scripts/setup_da3_trt.sh for TRT installation and model preparation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node


# (camera_type value, executable, node name)
# da_v3_trt uses a compound approach: relay + external TRT node (see DEPTH_DA3_INCLUDES)
DEPTH_SOURCES = [
    ('sim',       'depth_relay_node',     'depth_relay_node'),
    ('da_v3_trt', 'da3_relay_node',       'da3_relay_node'),
    ('realsense', 'realsense_relay_node', 'realsense_relay_node'),
    ('zed',       'zed_relay_node',       'zed_relay_node'),
]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('ugv_perception'), 'config', 'perception.yaml')

    use_sim_time      = LaunchConfiguration('use_sim_time')
    camera_type       = LaunchConfiguration('camera_type')
    perception_config = LaunchConfiguration('perception_config')

    def perception_node(executable, name, condition=None):
        return Node(
            package='ugv_perception',
            executable=executable,
            name=name,
            parameters=[perception_config, {'use_sim_time': use_sim_time}],
            output='screen',
            condition=condition,
        )

    depth_source_nodes = [
        perception_node(exe, name, IfCondition(EqualsSubstitution(camera_type, value)))
        for value, exe, name in DEPTH_SOURCES
    ]

    # When camera_type:=da_v3_trt, also launch the C++ TRT node that does inference.
    # This requires depth_anything_v3 to be built (needs TRT + CUDA toolkit installed).
    # The relay node (da3_relay_node) bridges TRT node output → canonical /perception/depth/*
    pkg_da3 = None
    try:
        pkg_da3 = get_package_share_directory('depth_anything_v3')
    except Exception:
        pass  # Package not built yet — user needs to install TRT and build it

    da3_trt_launch = []
    if pkg_da3:
        da3_params = os.path.join(
            get_package_share_directory('ugv_bringup'), 'config', 'da3_ugv.param.yaml')
        da3_trt_launch.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_da3, 'launch', 'depth_anything_v3.launch.py')
                ),
                launch_arguments={
                    'input_image_topic':       '/camera/image_raw',
                    'input_camera_info_topic': '/camera/camera_info',
                    'output_depth_topic':      '/depth_anything_v3/output/depth_image',
                    'output_point_cloud_topic': '/depth_anything_v3/output/point_cloud',
                    'params_file':             da3_params,
                }.items(),
                condition=IfCondition(EqualsSubstitution(camera_type, 'da_v3_trt'))
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('camera_type', default_value='sim',
                              description=(
                                  'Depth source — all publish identical /perception/depth/* topics:\n'
                                  '  sim        — Gazebo real depth_camera sensor (default for mode:=sim)\n'
                                  '  da_v3_trt  — Depth Anything V3 TensorRT (real-time GPU inference)\n'
                                  '               Requires: TRT + CUDA toolkit + ONNX model (see scripts/)\n'
                                  '  realsense  — Intel RealSense D435/D455 (hw mode)\n'
                                  '  zed        — Stereolabs ZED 2/ZED X (hw mode, outdoor)'
                              )),
        DeclareLaunchArgument('perception_config', default_value=default_config,
                              description='Perception parameter YAML'),

        *depth_source_nodes,
        *da3_trt_launch,
        perception_node('depth_to_pointcloud_node', 'depth_to_pointcloud_node'),
        perception_node('yolo_seg_node', 'yolo_seg_node'),
    ])
