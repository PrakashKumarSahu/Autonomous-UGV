import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_terrain')
    filters_config = os.path.join(pkg_share, 'config', 'grid_map_filters.yaml')
    vis_config = os.path.join(pkg_share, 'config', 'grid_map_visualization.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Official ANYbotics grid_map Filter Chain
    filter_node = Node(
        package='grid_map_demos',
        executable='filters_demo',
        name='grid_map_filters',
        output='screen',
        parameters=[filters_config, {'use_sim_time': use_sim_time}]
    )

    # 2. Official ANYbotics grid_map OccupancyGrid Visualization/Converter
    vis_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[vis_config, {'use_sim_time': use_sim_time}],
        remappings=[('traversability_grid', '/terrain/traversability_grid')]
    )

    # 3. Synchronized Traversability Bridge
    bridge_node = Node(
        package='ugv_terrain',
        executable='terrain_analysis_node',
        name='terrain_analysis_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        filter_node,
        vis_node,
        bridge_node
    ])
