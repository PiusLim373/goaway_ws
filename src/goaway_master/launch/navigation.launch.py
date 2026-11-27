"""Navigation half of nav2 for the Go2.

Ported from tmms_master (a fork of nav2_bringup's navigation_launch.py, Jazzy): no namespace,
ParameterFile instead of RewrittenYaml, use_sim_time pinned False. All ten servers are kept.

Velocity chain (the cmd_vel_nav remaps below):
    controller/behavior -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
                        -> collision_monitor -> /cmd_vel -> goaway_quadruped_controller (unpaused)
assisted_teleop is the exception: cmd_vel_teleop_out sends it to /quadruped_cmd_vel_teleop
(the UI's safety teleop, used while paused; see config/nav2_params.yaml).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile


def generate_launch_description():
    master_share = get_package_share_directory('goaway_master')
    default_params_file = os.path.join(master_share, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    container_name_full = ('/', container_name)

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'route_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
        'docking_server',
    ]

    configured_params = ParameterFile(params_file, allow_substs=True)

    # One process per node: use_composition:=False (debugging)
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', False),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_route',
                executable='route_server',
                name='route_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            # Last link to /cmd_vel. A source that never published zeroes every command
            # ("invalid source"): it reads /utlidar/scan.
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='opennav_docking',
                executable='opennav_docking',
                name='docking_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    # Everything into bringup.launch.py's nav2_container (the robot's path)
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', False),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=[('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_route',
                        plugin='nav2_route::RouteServer',
                        name='route_server',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=[('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_params],
                        remappings=[('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_collision_monitor',
                        plugin='nav2_collision_monitor::CollisionMonitor',
                        name='collision_monitor',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='opennav_docking',
                        plugin='opennav_docking::DockingServer',
                        name='docking_server',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                    ),
                ],
            ),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='nav2 parameters for the Go2'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Have the lifecycle manager configure+activate on startup'),
        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Load into the shared component container'),
        DeclareLaunchArgument(
            'container_name', default_value='nav2_container',
            description='Container to load into when use_composition is True'),
        DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='Respawn on crash (uncomposed only)'),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='log level'),

        load_nodes,
        load_composable_nodes,
    ])
