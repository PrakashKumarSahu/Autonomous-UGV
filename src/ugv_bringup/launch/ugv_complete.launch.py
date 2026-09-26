import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EqualsSubstitution

def generate_launch_description():
    pkg_bringup = get_package_share_directory('ugv_bringup')
    pkg_sim = get_package_share_directory('ugv_sim')
    pkg_terrain = get_package_share_directory('ugv_terrain')

    # Launch Configurations
    mode = LaunchConfiguration('mode', default='sim')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    enable_rtabmap = LaunchConfiguration('enable_rtabmap', default='true')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Launch mode: "sim" for Gazebo Harmonic, "hw" for physical robot'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz2 dashboard'
    )

    declare_rtabmap_cmd = DeclareLaunchArgument(
        'enable_rtabmap',
        default_value='true',
        description='Enable RTAB-Map SLAM'
    )

    # 1. Simulation Environment (only in sim mode)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'sim.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'sim'))
    )

    # 2. Perception AI (Depth Anything + YOLO Segmentation)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'perception.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. 2.5D Terrain Traversability Mapping (grid_map bridge)
    terrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_terrain, 'launch', 'terrain_mapping.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Visual Localization & Sensor Fusion (RTAB-Map + EKF)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'localization.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_rtabmap': enable_rtabmap
        }.items()
    )

    # 5. Autonomous Navigation (Nav2 with Smac Hybrid-A* & Regulated Pure Pursuit)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 6. Visualization Dashboard (RViz2)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'rviz.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(enable_rviz)
    )

    return LaunchDescription([
        declare_mode_cmd,
        declare_use_sim_time_cmd,
        declare_rviz_cmd,
        declare_rtabmap_cmd,
        sim_launch,
        perception_launch,
        terrain_launch,
        localization_launch,
        navigation_launch,
        rviz_launch
    ])
