"""Mapping: slam_toolbox (idle until a session starts) + mapping_manager.

slam_toolbox is a lifecycle node and is NOT configured here: mapping_manager configures and
activates it on ~/start_mapping and deactivates + cleans it up on ~/stop_mapping, so it uses no
CPU and publishes no map -> odom outside a session.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    master_share = get_package_share_directory('goaway_master')
    maps_dir = LaunchConfiguration('maps_dir')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(master_share, 'config', 'goaway_slam.yaml'),
                    {'use_sim_time': False}],
        output='screen')

    # respawn: it cleans up a session left over from a crash when it comes back
    mapping_manager = Node(
        package='mapping_utils',
        executable='mapping_manager_node.py',
        name='mapping_manager',
        parameters=[{'maps_dir': maps_dir}],
        respawn=True,
        respawn_delay=2.0,
        output='screen')

    return LaunchDescription([
        # tmms's map store: png/ (2D maps), slam/ (pose graphs)
        DeclareLaunchArgument('maps_dir', default_value='~/.htxgrrt/maps'),
        slam_toolbox,
        mapping_manager,
    ])
