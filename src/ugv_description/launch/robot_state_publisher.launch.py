import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'ugv.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            # Read joint states from relay node that renames SDF joints → URDF joints.
            # In sim mode: Gazebo bridge publishes wheel_left_joint/wheel_right_joint
            # but URDF defines left_wheel_joint/right_wheel_joint. The relay node
            # (joint_state_relay.py) outputs corrected names on /joint_states_urdf.
            ('joint_states', 'joint_states_urdf')
        ]
    )

    # joint_state_publisher is intentionally not created here.
    # In sim mode, ros_gz_bridge provides real /joint_states from Gazebo.
    # In hw mode, add a joint_state_publisher or joint_state_broadcaster as needed.


    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        # NOTE: joint_state_publisher is intentionally omitted in sim mode.
        # ros_gz_bridge publishes real joint states from Gazebo on /joint_states.
        # Running joint_state_publisher alongside causes duplicate publishers that
        # feed zeroed wheel positions to robot_state_publisher, making RViz show
        # wheels frozen at their neutral pose. For hw mode, add joint_state_publisher
        # back in a hw-conditional block if needed.
    ])
