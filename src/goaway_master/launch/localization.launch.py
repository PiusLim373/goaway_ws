"""Localization half of nav2 for the Go2: map_server + amcl.

Ported from tmms_master (a fork of nav2_bringup's localization_launch.py, Jazzy): no namespace,
use_sim_time pinned False, one map_server.

`map` defaults to config/startup_map.yaml, a blank throwaway map. AMCL only broadcasts
map -> odom once it has a map AND a pose (set_initial_pose: true in nav2_params.yaml); without
it global_costmap's activation times out after 60 s and the whole navigation half aborts.
A real map is loaded later through /map_server/load_map (first_map_only defaults to false).
Until then the AMCL pose means nothing.

Services: /map_server/load_map, /map_server/map; AMCL's are root-relative (/set_initial_pose,
/reinitialize_global_localization, /request_nomotion_update). /initialpose sets the pose.
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
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    container_name_full = ('/', container_name)

    lifecycle_nodes = ['map_server', 'amcl']

    configured_params = ParameterFile(params_file, allow_substs=True)

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', False),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'yaml_filename': map_yaml_file}],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', False),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[configured_params, {'yaml_filename': map_yaml_file}],
                    ),
                    ComposableNode(
                        package='nav2_amcl',
                        plugin='nav2_amcl::AmclNode',
                        name='amcl',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_localization',
                        parameters=[{'autostart': autostart, 'node_names': lifecycle_nodes}],
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
        # Keep in a replacement: free space, (0,0) inside, fully opaque PNG (alpha < 255 = unknown)
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(master_share, 'config', 'startup_map.yaml'),
            description='Map yaml loaded at startup (blank throwaway by default)'),
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
