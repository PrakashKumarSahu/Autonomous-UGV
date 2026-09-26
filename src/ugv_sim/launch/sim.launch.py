import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ugv_sim = get_package_share_directory('ugv_sim')
    pkg_ugv_description = get_package_share_directory('ugv_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_ugv_sim, 'worlds', 'outdoor_unstructured.sdf'),
        description='Path to the Gazebo SDF world file'
    )

    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config',
        default_value=os.path.join(pkg_ugv_sim, 'config', 'gazebo_bridge.yaml'),
        description='Path to gazebo_bridge.yaml'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # 1. Robot State Publisher (URDF & TF)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ugv_description, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 2. Gazebo Harmonic Simulation Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [LaunchConfiguration('world'), ' -r']}.items()
    )

    # 3. Spawn UGV Entity into Gazebo
    spawn_ugv = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'ugv', '-z', '0.2'],
        output='screen'
    )

    # 4. ROS-GZ Parameter Bridge
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
        robot_state_publisher,
        gz_sim,
        spawn_ugv,
        ros_gz_bridge
    ])
