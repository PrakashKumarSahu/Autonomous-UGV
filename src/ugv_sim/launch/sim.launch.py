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

    # Path to existing Tugbot in Warehouse world
    default_world = os.path.expanduser(
        '~/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/tugbot in warehouse/2/tugbot_warehouse.sdf'
    )
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
    gz_resource_path = os.path.expanduser(
        '~/.gz/fuel/fuel.gazebosim.org/:~/.gz/fuel/fuel.ignitionrobotics.org/movai/models/:~/.gz/models/'
    )
    set_gz_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path)

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

    return LaunchDescription([
        world_arg,
        bridge_config_arg,
        use_sim_time_arg,
        set_gz_resource_path,
        robot_state_publisher,
        gz_sim,
        ros_gz_bridge
    ])
