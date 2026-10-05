"""
terrain_mapping.launch.py — 2.5D terrain traversability mapping.

Runs terrain_analysis_node, which fuses /perception/depth/points with the YOLO26
hazard mask into /terrain/traversability_grid (OccupancyGrid, base_footprint).
Parameters: ugv_terrain/config/terrain.yaml (override with terrain_config:=...).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('ugv_terrain'), 'config', 'terrain.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('terrain_config', default_value=default_config,
                              description='terrain_analysis_node parameter YAML'),
        Node(
            package='ugv_terrain',
            executable='terrain_analysis_node',
            name='terrain_analysis_node',
            parameters=[LaunchConfiguration('terrain_config'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
    ])
