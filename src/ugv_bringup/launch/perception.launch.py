"""
perception.launch.py — Perception stack (depth source + point cloud + YOLO26).

All node parameters live in ugv_perception/config/perception.yaml; this file only
decides WHICH nodes run. Override the config without rebuilding:
    perception_config:=/path/to/my_perception.yaml

Depth source (exactly one, chosen by camera_type):
    sim        → depth_relay_node      (Gazebo depth_camera)
    monocular  → depth_node            (Depth Anything V2 Metric, RGB only)
    realsense  → realsense_relay_node  (Intel RealSense D435/D455)
    zed        → zed_relay_node        (Stereolabs ZED 2 / ZED X)
Always on:
    depth_to_pointcloud_node  /perception/depth/image_raw → /perception/depth/points
    yolo_seg_node             /camera/image_raw → /perception/hazard_mask + overlay
Optional:
    depth_viz_node            (enable_depth_viz:=true) AI depth overlay for RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node


# (camera_type value, executable, node name)
DEPTH_SOURCES = [
    ('sim',       'depth_relay_node',     'depth_relay_node'),
    ('monocular', 'depth_node',           'depth_node'),
    ('realsense', 'realsense_relay_node', 'realsense_relay_node'),
    ('zed',       'zed_relay_node',       'zed_relay_node'),
]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('ugv_perception'), 'config', 'perception.yaml')

    use_sim_time      = LaunchConfiguration('use_sim_time')
    camera_type       = LaunchConfiguration('camera_type')
    enable_depth_viz  = LaunchConfiguration('enable_depth_viz')
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('camera_type', default_value='sim',
                              description='Depth source: sim | monocular | realsense | zed'),
        DeclareLaunchArgument('enable_depth_viz', default_value='false',
                              description='Run Depth Anything V2 as an RViz-only AI depth overlay '
                                          '(/perception/depth_ai/colorized, ~2 GB VRAM)'),
        DeclareLaunchArgument('perception_config', default_value=default_config,
                              description='Perception parameter YAML'),

        *depth_source_nodes,
        perception_node('depth_to_pointcloud_node', 'depth_to_pointcloud_node'),
        perception_node('yolo_seg_node', 'yolo_seg_node'),
        perception_node('depth_node', 'depth_viz_node', IfCondition(enable_depth_viz)),
    ])
