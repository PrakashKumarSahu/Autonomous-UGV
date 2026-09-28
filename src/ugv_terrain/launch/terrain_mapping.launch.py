import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_terrain')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── Terrain Traversability Analysis ───────────────────────────────────────
    # Subscribes to /perception/depth/points (camera PointCloud2) and
    # /perception/hazard_mask (YOLO semantic mask).
    # Publishes /terrain/traversability_grid (OccupancyGrid, frame: base_footprint)
    # Robot-relative rolling grid: free=0, obstacle=100, unknown=-1.
    # Used by terrain visualization in RViz.
    terrain_analysis_node = Node(
        package='ugv_terrain',
        executable='terrain_analysis_node',
        name='terrain_analysis_node',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'pointcloud_topic':   '/perception/depth/points',
            'hazard_mask_topic':  '/perception/hazard_mask',
            'output_grid_topic':  '/terrain/traversability_grid',
            'map_frame':          'odom',
            'base_frame':         'base_footprint',
            'grid_resolution':    0.10,
            'grid_size_x':        12.0,
            'grid_size_y':        12.0,
            'max_step_height':    0.22,
            'max_slope_angle':    25.0,
        }],
        output='screen'
    )

    # NOTE: The ANYbotics grid_map filter chain (filters_demo, grid_map_visualization)
    # has been removed. Those nodes expect a grid_map_msgs/GridMap message with an
    # "elevation" layer on /grid_map — which requires a 3D lidar or elevation map
    # source. Since this UGV is camera-only, the filter chain received no input and
    # was running idle. The terrain_analysis_node produces a Nav2-compatible
    # OccupancyGrid directly from the camera depth + YOLO hazard mask.

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'
        ),
        terrain_analysis_node,
    ])
