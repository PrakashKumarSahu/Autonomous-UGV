import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable, OpaqueFunction, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ══════════════════════════════════════════════════════════════════════════════
# DESIGN NOTES — WHY OpaqueFunction?
# ══════════════════════════════════════════════════════════════════════════════
# The GZ bridge YAML needs world_name and robot_name substituted into the topic
# paths at launch time (e.g. /world/{world_name}/model/{robot_name}/...).
# LaunchConfiguration values are not Python strings at describe-time — they are
# substitution objects resolved only when the launch actually runs.
# OpaqueFunction defers execution to run-time, where .perform(context) resolves
# them to actual strings. We then generate the bridge YAML dynamically and write
# it to a temporary file that the bridge node reads via the config_file parameter.
#
# This eliminates the hardcoded 'world_demo' / 'localbot' strings from the YAML,
# making the sim launch work with ANY Gazebo world name and ANY robot model.
# ══════════════════════════════════════════════════════════════════════════════


def _make_bridge_yaml(world_name: str, robot_name: str) -> str:
    """Generate ros_gz_bridge YAML config from world_name and robot_name."""
    gz = f'/world/{world_name}/model/{robot_name}'
    m  = f'/model/{robot_name}'

    config = [
        # ── RGB Camera ─────────────────────────────────────────────────────
        {
            'ros_topic_name': '/camera/image_raw',
            'gz_topic_name':  f'{gz}/link/camera_link/sensor/color/image',
            'ros_type_name':  'sensor_msgs/msg/Image',
            'gz_type_name':   'gz.msgs.Image',
            'direction':      'GZ_TO_ROS',
        },
        {
            'ros_topic_name': '/camera/camera_info',
            'gz_topic_name':  f'{gz}/link/camera_link/sensor/color/camera_info',
            'ros_type_name':  'sensor_msgs/msg/CameraInfo',
            'gz_type_name':   'gz.msgs.CameraInfo',
            'direction':      'GZ_TO_ROS',
        },
        # ── Depth Camera (CRITICAL: suffix is "depth_image" not "image") ──
        {
            'ros_topic_name': '/camera/depth/image_raw',
            'gz_topic_name':  f'{gz}/link/camera_link/sensor/depth/depth_image',
            'ros_type_name':  'sensor_msgs/msg/Image',
            'gz_type_name':   'gz.msgs.Image',
            'direction':      'GZ_TO_ROS',
        },
        {
            'ros_topic_name': '/camera/depth/camera_info',
            'gz_topic_name':  f'{gz}/link/camera_link/sensor/depth/camera_info',
            'ros_type_name':  'sensor_msgs/msg/CameraInfo',
            'gz_type_name':   'gz.msgs.CameraInfo',
            'direction':      'GZ_TO_ROS',
        },
        # ── IMU ────────────────────────────────────────────────────────────
        {
            'ros_topic_name': '/imu/data',
            'gz_topic_name':  f'{gz}/link/imu_link/sensor/imu/imu',
            'ros_type_name':  'sensor_msgs/msg/Imu',
            'gz_type_name':   'gz.msgs.IMU',
            'direction':      'GZ_TO_ROS',
        },
        # ── Wheel Odometry (DiffDrive) ─────────────────────────────────────
        {
            'ros_topic_name': '/odometry/filtered',
            'gz_topic_name':  f'{m}/odometry',
            'ros_type_name':  'nav_msgs/msg/Odometry',
            'gz_type_name':   'gz.msgs.Odometry',
            'direction':      'GZ_TO_ROS',
        },
        # ── TF (odom → base_footprint) ─────────────────────────────────────
        {
            'ros_topic_name': '/tf',
            'gz_topic_name':  f'{m}/tf',
            'ros_type_name':  'tf2_msgs/msg/TFMessage',
            'gz_type_name':   'gz.msgs.Pose_V',
            'direction':      'GZ_TO_ROS',
        },
        # ── Simulation Clock ───────────────────────────────────────────────
        {
            'ros_topic_name': '/clock',
            'gz_topic_name':  f'/world/{world_name}/clock',
            'ros_type_name':  'rosgraph_msgs/msg/Clock',
            'gz_type_name':   'gz.msgs.Clock',
            'direction':      'GZ_TO_ROS',
        },
        # ── Joint States (QoS-bridged by joint_state_relay → RSP) ─────────
        {
            'ros_topic_name': '/joint_states',
            'gz_topic_name':  f'/world/{world_name}/model/{robot_name}/joint_state',
            'ros_type_name':  'sensor_msgs/msg/JointState',
            'gz_type_name':   'gz.msgs.Model',
            'direction':      'GZ_TO_ROS',
        },
        # ── Command Velocity (Nav2 → Gazebo DiffDrive) ─────────────────────
        {
            'ros_topic_name': '/cmd_vel',
            'gz_topic_name':  f'{m}/cmd_vel',
            'ros_type_name':  'geometry_msgs/msg/Twist',
            'gz_type_name':   'gz.msgs.Twist',
            'direction':      'ROS_TO_GZ',
        },
    ]

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='_gz_bridge.yaml', prefix='ugv_', delete=False
    )
    yaml.dump(config, tmp, default_flow_style=False)
    tmp_path = tmp.name
    tmp.close()
    return tmp_path


def launch_setup(context, *args, **kwargs):
    """OpaqueFunction: resolved at launch runtime so LaunchConfigurations are strings."""
    world_name = LaunchConfiguration('world_name').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    spawn_x    = LaunchConfiguration('spawn_x').perform(context)
    spawn_y    = LaunchConfiguration('spawn_y').perform(context)
    spawn_z    = LaunchConfiguration('spawn_z').perform(context)
    spawn_yaw  = LaunchConfiguration('spawn_yaw').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    pkg_ugv_sim = get_package_share_directory('ugv_sim')
    robot_sdf   = os.path.join(pkg_ugv_sim, 'models', 'localbot', 'model.sdf')

    # ── 1. Dynamic GZ bridge YAML ─────────────────────────────────────────────
    # Written to a temp file at launch time — parametric on world_name + robot_name.
    bridge_yaml_path = _make_bridge_yaml(world_name, robot_name)

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_yaml_path}],
        output='screen'
    )

    # ── 2. Robot Spawner ──────────────────────────────────────────────────────
    # Spawns localbot into the already-running Gazebo world via the EntityFactory
    # service. Wrapped in a 3-second delay so Gazebo physics is ready.
    # Use robot_sdf to load the exact same model regardless of world file content.
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-world', world_name,
            '-name',  robot_name,
            '-file',  robot_sdf,
            '-x',     spawn_x,
            '-y',     spawn_y,
            '-z',     spawn_z,
            '-Y',     spawn_yaw,
        ],
        output='screen'
    )

    # ── 3. Joint State Relay (QoS bridge: BestEffort → Reliable) ─────────────
    # Gazebo bridge publishes /joint_states as BestEffort; robot_state_publisher
    # needs Reliable. relay also filters to URDF wheel joints only.
    # With Localbot, joint names already match URDF — relay does QoS-only.
    relay_node = Node(
        package='ugv_sim',
        executable='joint_state_relay.py',
        name='joint_state_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time.lower() in ('true', '1')}]
    )

    return [
        bridge_node,
        TimerAction(period=3.0, actions=[spawn_node]),  # wait for Gazebo physics
        relay_node,
    ]


def generate_launch_description():
    pkg_ugv_sim         = get_package_share_directory('ugv_sim')
    pkg_ugv_description = get_package_share_directory('ugv_description')
    pkg_ros_gz_sim      = get_package_share_directory('ros_gz_sim')

    default_world = os.path.join(pkg_ugv_sim, 'worlds', 'ugv_test_arena.sdf')

    # ── Environment variables ─────────────────────────────────────────────────
    # GZ_SIM_RESOURCE_PATH: local install models come FIRST so model://localbot
    # resolves to our project files before any Fuel cache.
    local_models_path = os.path.join(pkg_ugv_sim, 'models')
    gz_resource_path  = (
        local_models_path + ':' +
        os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org/') + ':' +
        os.path.expanduser('~/.gz/fuel/fuel.ignitionrobotics.org/movai/models/') + ':' +
        os.path.expanduser('~/.gz/models/')
    )

    return LaunchDescription([
        # ── Declare args ──────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description=(
                'Path to a Gazebo SDF world file. Any world works as long as it '
                'does NOT include the robot model (robot is spawned separately). '
                'Built-in worlds: ugv_test_arena.sdf | empty_world.sdf'
            )
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='world_demo',
            description=(
                'Name of the Gazebo world — must match the <world name="..."> '
                'attribute inside the SDF file. Used to construct GZ sensor topic '
                'paths: /world/{world_name}/model/{robot_name}/...'
            )
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='localbot',
            description=(
                'Name given to the spawned robot entity in Gazebo. '
                'Used in GZ topic paths and EntityFactory spawn service.'
            )
        ),
        DeclareLaunchArgument(
            'spawn_x',   default_value='0.0',
            description='Robot spawn X position in world (metres)'
        ),
        DeclareLaunchArgument(
            'spawn_y',   default_value='-6.5',
            description='Robot spawn Y position in world (metres)'
        ),
        DeclareLaunchArgument(
            'spawn_z',   default_value='0.01',
            description='Robot spawn Z position (metres). 0.01 = just above ground.'
        ),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='1.5708',
            description='Robot spawn yaw angle (radians). 1.5708=north, 0=east, 3.14159=south.'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo /clock for all ROS nodes'
        ),

        # ── Environment setup ─────────────────────────────────────────────────
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        # Prevents Qt rendering issues on NVIDIA + multi-GPU systems
        SetEnvironmentVariable('QSG_RENDER_LOOP',           'basic'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM',          '1'),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD',  '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME',  'nvidia'),

        # ── Robot State Publisher (URDF → /tf_static + /tf) ──────────────────
        # Must start before bridge so RSP's static TF is available when sensors
        # arrive. RSP reads /joint_states_urdf (from joint_state_relay).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ugv_description, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        ),

        # ── Gazebo Harmonic ───────────────────────────────────────────────────
        # gz_args: path to world SDF + '-r' flag (run immediately, not paused).
        # World SDF contains the environment; robot is spawned separately below.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': [LaunchConfiguration('world'), ' -r']
            }.items()
        ),

        # ── Dynamic bridge + spawner + relay (OpaqueFunction) ─────────────────
        # Resolved at launch runtime so world_name / robot_name are real strings.
        OpaqueFunction(function=launch_setup),
    ])
