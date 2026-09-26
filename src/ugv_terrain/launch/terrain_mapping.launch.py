import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_terrain')
    config_file = os.path.join(pkg_share, 'config', 'grid_map_fusion.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    terrain_node = Node(
        package='ugv_terrain',
        executable='terrain_analysis_node',
        name='terrain_analysis_node',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        terrain_node
    ])
