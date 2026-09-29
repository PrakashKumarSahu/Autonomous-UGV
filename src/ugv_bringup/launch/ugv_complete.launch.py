import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EqualsSubstitution

# ══════════════════════════════════════════════════════════════════════════════
# ugv_complete.launch.py — Top-Level System Launcher
# ══════════════════════════════════════════════════════════════════════════════
#
# Single entry point for the entire Autonomous UGV system:
#   Simulation → Perception → Terrain → SLAM → Navigation → RViz
#
# QUICK START:
#   ros2 launch ugv_bringup ugv_complete.launch.py          # defaults (sim mode)
#   ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=realsense
#
# CUSTOM WORLD (any SDF file, no robot inside):
#   ros2 launch ugv_bringup ugv_complete.launch.py \
#     world:=/path/to/my_world.sdf \
#     world_name:=my_world \
#     spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0
#
# ARGUMENT REFERENCE (all args with defaults and descriptions below)
# ══════════════════════════════════════════════════════════════════════════════


def generate_launch_description():
    pkg_bringup = get_package_share_directory('ugv_bringup')
    pkg_sim     = get_package_share_directory('ugv_sim')
    pkg_terrain = get_package_share_directory('ugv_terrain')

    default_world = os.path.join(pkg_sim, 'worlds', 'ugv_test_arena.sdf')

    # ── Launch Configurations ─────────────────────────────────────────────────
    mode          = LaunchConfiguration('mode')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    enable_rviz   = LaunchConfiguration('enable_rviz')
    camera_type   = LaunchConfiguration('camera_type')
    world         = LaunchConfiguration('world')
    world_name    = LaunchConfiguration('world_name')
    robot_name    = LaunchConfiguration('robot_name')
    spawn_x       = LaunchConfiguration('spawn_x')
    spawn_y       = LaunchConfiguration('spawn_y')
    spawn_z       = LaunchConfiguration('spawn_z')
    spawn_yaw     = LaunchConfiguration('spawn_yaw')

    return LaunchDescription([
        # ══════════════════════════════════════════════════════════════════════
        # ARGUMENTS
        # ══════════════════════════════════════════════════════════════════════

        # ── System mode ───────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'mode', default_value='sim',
            description=(
                'Launch mode:\n'
                '  sim — Gazebo simulation (default)\n'
                '  hw  — Physical robot (skips Gazebo, uses real sensors)'
            )
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description=(
                'Use Gazebo /clock for all ROS nodes. '
                'Set false for hw mode (uses wall clock).'
            )
        ),

        # ── World configuration (sim mode only) ───────────────────────────────
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description=(
                'Path to a Gazebo SDF world file.\n'
                'The world must NOT contain the robot model — it is spawned separately.\n'
                'Built-in worlds:\n'
                '  ugv_test_arena.sdf  — obstacle course (default)\n'
                '  empty_world.sdf     — flat ground only, bring your own obstacles\n'
                'Custom: point to any /path/to/your_world.sdf'
            )
        ),
        DeclareLaunchArgument(
            'world_name', default_value='world_demo',
            description=(
                'Name attribute of the Gazebo world — must match <world name="..."> in the SDF.\n'
                'Used to construct GZ sensor topic paths:\n'
                '  /world/{world_name}/model/{robot_name}/link/camera_link/...'
            )
        ),

        # ── Robot configuration ────────────────────────────────────────────────
        DeclareLaunchArgument(
            'robot_name', default_value='localbot',
            description=(
                'Entity name for the spawned robot in Gazebo.\n'
                'Used in all GZ topic paths and the EntityFactory spawn service.\n'
                'Also determines the Teleop GUI topic: /model/{robot_name}/cmd_vel'
            )
        ),

        # ── Spawn position (where the robot starts in the world) ──────────────
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0',
            description='Robot initial X position in world frame (metres, East)'
        ),
        DeclareLaunchArgument(
            'spawn_y', default_value='-6.5',
            description='Robot initial Y position in world frame (metres, North)'
        ),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.01',
            description='Robot initial Z position (metres). 0.01 gives ~1cm ground clearance.'
        ),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='1.5708',
            description=(
                'Robot initial yaw angle (radians).\n'
                '  1.5708 = facing north (+Y)  [default for ugv_test_arena]\n'
                '  0.0    = facing east  (+X)\n'
                '  3.1416 = facing south (-Y)'
            )
        ),

        # ── Perception / depth source ──────────────────────────────────────────
        DeclareLaunchArgument(
            'camera_type', default_value='sim',
            description=(
                'Depth source — swappable without changing anything downstream:\n'
                '  sim        — Gazebo real depth_camera sensor (default)\n'
                '  monocular  — Depth Anything V3 (any RGB-only camera)\n'
                '  realsense  — Intel RealSense D435/D455 (hw mode)\n'
                '  zed        — Stereolabs ZED 2/ZED X (hw mode, outdoor)\n'
                'All sources publish identical /perception/depth/* topics.'
            )
        ),
        DeclareLaunchArgument(
            'enable_depth_viz', default_value='false',
            description=(
                'Run Depth Anything V3 as a RViz visualization overlay.\n'
                'Does NOT affect navigation — purely for visual comparison.\n'
                'Disabled by default (saves GPU compute).'
            )
        ),

        # ── SLAM ──────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'enable_rtabmap', default_value='true',
            description='Enable RTAB-Map visual SLAM (publishes /map and map→odom TF)'
        ),

        # ── Visualization ──────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='Launch RViz2 visualization dashboard'
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(pkg_bringup, 'rviz', 'ugv_autonomy.rviz'),
            description='Path to RViz2 .rviz config file'
        ),

        # ── Nav2 configuration ────────────────────────────────────────────────
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(pkg_bringup, 'config', 'nav2_params.yaml'),
            description=(
                'Path to Nav2 parameters YAML file.\n'
                'Override to use a custom Nav2 config without rebuilding.'
            )
        ),

        # ══════════════════════════════════════════════════════════════════════
        # MODULES (launched in dependency order)
        # ══════════════════════════════════════════════════════════════════════

        # ── 1. Simulation Environment (sim mode only) ─────────────────────────
        # Starts Gazebo, spawns localbot, bridges all GZ↔ROS topics.
        # Also starts robot_state_publisher (URDF → TF).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sim, 'launch', 'sim.launch.py')
            ),
            launch_arguments={
                'world':       world,
                'world_name':  world_name,
                'robot_name':  robot_name,
                'spawn_x':     spawn_x,
                'spawn_y':     spawn_y,
                'spawn_z':     spawn_z,
                'spawn_yaw':   spawn_yaw,
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(EqualsSubstitution(mode, 'sim'))
        ),

        # ── 2. Perception Stack ───────────────────────────────────────────────
        # Depth source (camera_type selects one) + YOLO + point cloud.
        # Output: /perception/depth/*, /perception/yolo/*, /perception/depth/points
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'perception.launch.py')
            ),
            launch_arguments={
                'use_sim_time':      use_sim_time,
                'camera_type':       camera_type,
                'enable_depth_viz':  LaunchConfiguration('enable_depth_viz'),
            }.items()
        ),

        # ── 3. Terrain Traversability Mapping ─────────────────────────────────
        # 2.5D height map from depth + YOLO hazard fusion.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_terrain, 'launch', 'terrain_mapping.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # ── 4. Visual SLAM + Sensor Fusion ────────────────────────────────────
        # RTAB-Map: /map + map→odom TF. Optional EKF fusion in hw mode.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'use_sim_time':    use_sim_time,
                'enable_rtabmap':  LaunchConfiguration('enable_rtabmap'),
                'mode':            mode,
            }.items()
        ),

        # ── 5. Autonomous Navigation (Nav2) ───────────────────────────────────
        # Smac2D planner + Regulated Pure Pursuit controller + costmaps.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'use_sim_time':      use_sim_time,
                'nav2_params_file':  LaunchConfiguration('nav2_params_file'),
            }.items()
        ),

        # ── 6. RViz2 Visualization Dashboard ──────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'rviz.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz_config':  LaunchConfiguration('rviz_config'),
            }.items(),
            condition=IfCondition(enable_rviz)
        ),
    ])
