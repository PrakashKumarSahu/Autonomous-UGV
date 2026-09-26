from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    depth_node = Node(
        package='ugv_perception',
        executable='depth_node',
        name='depth_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    yolo_seg_node = Node(
        package='ugv_perception',
        executable='yolo_seg_node',
        name='yolo_seg_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        depth_node,
        yolo_seg_node
    ])
