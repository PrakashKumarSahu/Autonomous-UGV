import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ugv_sim = get_package_share_directory('ugv_sim')
    pkg_ugv_description = get_package_share_directory('ugv_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to Tugbot in Warehouse world located inside ugv_sim
    default_world = os.path.join(pkg_ugv_sim, 'worlds', 'tugbot_warehouse.sdf')
    default_bridge_config = os.path.join(pkg_ugv_sim, 'config', 'gazebo_bridge.yaml')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to the Gazebo SDF world file'
    )

    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config',
        default_value=default_bridge_config,
        description='Path to gazebo_bridge.yaml'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Configure Gazebo Resource Path so all Tugbot and Warehouse models are discovered
    local_models_path = os.path.join(pkg_ugv_sim, 'models')
    gz_resource_path = local_models_path + ':' + os.path.expanduser(
        '~/.gz/fuel/fuel.gazebosim.org/:~/.gz/fuel/fuel.ignitionrobotics.org/movai/models/:~/.gz/models/'
    )
    set_gz_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path)
    set_qsg_render = SetEnvironmentVariable('QSG_RENDER_LOOP', 'basic')
    set_qt_mitshm = SetEnvironmentVariable('QT_X11_NO_MITSHM', '1')
    set_nv_prime = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    set_nv_glx = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')

    # 1. Robot State Publisher (URDF & TF)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ugv_description, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 2. Gazebo Harmonic Simulation Launch (Loads existing Tugbot in Warehouse world)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [LaunchConfiguration('world'), ' -r']}.items()
    )

    # 3. ROS-GZ Parameter Bridge for Tugbot Camera, IMU, Odometry, and Cmd_Vel
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': LaunchConfiguration('bridge_config')}],
        output='screen'
    )

    # 4. Joint State Relay: renames Gazebo SDF joint names to URDF names
    # SDF Tugbot model uses: wheel_left_joint / wheel_right_joint
    # URDF robot model uses: left_wheel_joint / right_wheel_joint
    # Without this relay, robot_state_publisher cannot publish TF for wheel links,
    # causing RViz "No transform from [left_wheel_link]" / "No transform from [right_wheel_link]"
    joint_state_relay = Node(
        package='ugv_sim',
        executable='joint_state_relay.py',
        name='joint_state_relay',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    return LaunchDescription([
        world_arg,
        bridge_config_arg,
        use_sim_time_arg,
        set_gz_resource_path,
        set_qsg_render,
        set_qt_mitshm,
        set_nv_prime,
        set_nv_glx,
        robot_state_publisher,
        gz_sim,
        ros_gz_bridge,
        joint_state_relay
    ])
