import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EqualsSubstitution

def generate_launch_description():
    pkg_bringup = get_package_share_directory('ugv_bringup')
    pkg_sim     = get_package_share_directory('ugv_sim')
    pkg_terrain = get_package_share_directory('ugv_terrain')

    # ── Launch Configurations ─────────────────────────────────────────────────
    mode             = LaunchConfiguration('mode',             default='sim')
    use_sim_time     = LaunchConfiguration('use_sim_time',     default='true')
    enable_rviz      = LaunchConfiguration('enable_rviz',      default='true')
    enable_rtabmap   = LaunchConfiguration('enable_rtabmap',   default='true')
    camera_type      = LaunchConfiguration('camera_type',      default='sim')
    enable_depth_viz = LaunchConfiguration('enable_depth_viz', default='false')

    return LaunchDescription([
        # ── Declare args ──────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'mode', default_value='sim',
            description='Launch mode: "sim" (Gazebo) | "hw" (physical robot)'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock when mode:=sim'
        ),
        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='Launch RViz2 visualization dashboard'
        ),
        DeclareLaunchArgument(
            'enable_rtabmap', default_value='true',
            description='Enable RTAB-Map visual SLAM'
        ),
        DeclareLaunchArgument(
            'camera_type', default_value='sim',
            description=(
                'Depth source — swappable without changing anything else:\n'
                '  sim        — Gazebo real depth_camera sensor (default)\n'
                '  monocular  — Depth Anything V3 (any RGB-only camera)\n'
                '  realsense  — Intel RealSense D435/D455 (hw mode)\n'
                '  zed        — Stereolabs ZED 2/ZED X (hw mode, outdoor)\n'
                'All sources publish to /perception/depth/image_raw (32FC1, metres).'
            )
        ),
        DeclareLaunchArgument(
            'enable_depth_viz', default_value='false',
            description='Run Depth Anything V3 as visualization overlay (RViz only, not Nav2)'
        ),

        # ── 1. Simulation Environment (sim mode only) ─────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sim, 'launch', 'sim.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(EqualsSubstitution(mode, 'sim'))
        ),

        # ── 2. Perception AI Stack ────────────────────────────────────────────
        # Depth source (camera_type selects: sim/monocular/realsense/zed)
        # + YOLO segmentation
        # + depth_image_proc point cloud generator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'perception.launch.py')
            ),
            launch_arguments={
                'use_sim_time':     use_sim_time,
                'camera_type':      camera_type,
                'enable_depth_viz': enable_depth_viz,
            }.items()
        ),

        # ── 3. 2.5D Terrain Traversability Mapping ────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_terrain, 'launch', 'terrain_mapping.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # ── 4. Visual SLAM + Sensor Fusion (RTAB-Map + optional EKF) ─────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'use_sim_time':   use_sim_time,
                'enable_rtabmap': enable_rtabmap,
                'mode':           mode,
            }.items()
        ),

        # ── 5. Autonomous Navigation (Nav2: Smac2D + Regulated Pure Pursuit) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # ── 6. RViz2 Visualization Dashboard ─────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'rviz.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(enable_rviz)
        ),
    ])
