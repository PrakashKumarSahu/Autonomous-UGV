import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ugv_perception = get_package_share_directory('ugv_perception')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Depth Anything V3 / V2 Monocular Depth Node
    depth_node = Node(
        package='ugv_perception',
        executable='depth_node',
        name='depth_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 2. Official ROS 2 depth_image_proc PointCloud Generator
    depth_image_proc_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='point_cloud_xyz_node',
        output='screen',
        remappings=[
            ('image_rect', '/perception/depth/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('points', '/perception/depth/points')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. YOLO Instance Segmentation Node (Ultralytics YOLOv8/YOLO11)
    yolo_seg_node = Node(
        package='ugv_perception',
        executable='yolo_seg_node',
        name='yolo_seg_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_image_topic': '/camera/image_raw',
            'output_mask_topic': '/perception/hazard_mask'
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        depth_node,
        depth_image_proc_node,
        yolo_seg_node
    ])
