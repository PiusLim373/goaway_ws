"""nav2 for the Go2: the component container, then both halves.

Ported from tmms_master. Replaces nav2_bringup's bringup_launch.py, which always loads its own
localization/navigation launches from nav2_bringup's share directory.

    localization.launch.py  -- map_server, amcl, lifecycle_manager_localization
    navigation.launch.py    -- the ten navigation nodes + lifecycle_manager_navigation

Standalone, for debugging (per-node logs, CPU, respawn):
    ros2 launch goaway_master bringup.launch.py use_composition:=False use_respawn:=True
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    master_share = get_package_share_directory('goaway_master')
    launch_dir = os.path.join(master_share, 'launch')
    default_params_file = os.path.join(master_share, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    configured_params = ParameterFile(params_file, allow_substs=True)

    common_args = {
        'params_file': params_file,
        'autostart': autostart,
        'use_composition': use_composition,
        'use_respawn': use_respawn,
        'log_level': log_level,
        'container_name': 'nav2_container',
    }

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='nav2 parameters for the Go2'),
        # One process (one DDS participant) for all of nav2 on the robot
        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Load nav2 into a single component container'),
        DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='Respawn on crash (uncomposed only)'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Have the lifecycle managers configure+activate on startup'),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='log level'),

        # Isolated: each component has its own executor, so one busy server can't starve another
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization.launch.py')),
            launch_arguments=common_args.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
            launch_arguments=common_args.items()),
    ])
