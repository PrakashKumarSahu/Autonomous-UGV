import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('ugv_bringup')
    default_nav2_params = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')

    use_sim_time     = LaunchConfiguration('use_sim_time')
    autostart        = LaunchConfiguration('autostart')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    # Nav2 lifecycle nodes managed by lifecycle_manager
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behaviors',
        'bt_navigator'
    ]

    # ── Controller Server (Regulated Pure Pursuit) ────────────────────────────
    # Follows the planned path at 20 Hz using /odometry/filtered and /tf.
    # Config: nav2_params.yaml → controller_server → FollowPath (RPP plugin).
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odom',    '/odometry/filtered')
        ]
    )

    # ── Planner Server (Smac2D) ───────────────────────────────────────────────
    # Global path planning on /global_costmap (built from /map).
    # Config: nav2_params.yaml → planner_server → GridBased (SmacPlanner2D plugin).
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    # ── Behavior Server (Spin/Backup/Wait) ────────────────────────────────────
    # Recovery behaviors activated by the BT navigator when the robot is stuck.
    behaviors_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behaviors',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    # ── BT Navigator (goal orchestration) ────────────────────────────────────
    # Accepts NavigateToPose actions and orchestrates the full Nav2 pipeline
    # via a Behavior Tree (BT). Triggers planner → controller → recovery loop.
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    # ── Lifecycle Manager ─────────────────────────────────────────────────────
    # Manages Nav2 node lifecycle (configure → activate → deactivate → cleanup).
    # autostart=true: automatically transitions all nodes to active state.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart':    autostart,
            'node_names':   lifecycle_nodes
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock (/clock from Gazebo)'
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Auto-activate all Nav2 lifecycle nodes at startup'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params,
            description=(
                'Path to Nav2 parameters YAML.\n'
                'Override without rebuilding:\n'
                '  nav2_params_file:=/path/to/my_nav2_params.yaml'
            )
        ),
        controller_node,
        planner_node,
        behaviors_node,
        bt_navigator_node,
        lifecycle_manager,
    ])
