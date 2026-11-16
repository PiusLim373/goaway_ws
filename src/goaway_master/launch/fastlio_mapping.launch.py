import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# FAST_LIO and its L1 cloud merger, started by operation.launch.py and always on:
# goaway_odometry takes over from FAST_LIO's /Odometry when Unitree's odom stops (HTX).
# mapping:=false (the default) turns the map, scan
# and path outputs off, which would otherwise grow without end. Until mapping can be switched
# at runtime, map by restarting operation.launch.py with fastlio_mapping:=true.
#
# FAST_LIO publishes camera_init -> body (body is the IMU, utlidar_imu). goaway_odometry
# places camera_init in odom; body is not attached to base_link (base_link already has a parent).


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('goaway_master'), 'config', 'goaway_fastlio.yaml')

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    mapping = DeclareLaunchArgument('mapping', default_value='false')
    # L1 messages merged per FAST_LIO scan (1 = as they come)
    merge_count = DeclareLaunchArgument('merge_count', default_value='3')
    map_on = ParameterValue(LaunchConfiguration('mapping'), value_type=bool)
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    # /utlidar/cloud -> /utlidar/cloud_merged (FAST_LIO's lid_topic): each L1 message only
    # sees part of the room.
    merger = Node(
        package='mapping_utils',
        executable='lidar_merger_node',
        name='lidar_merger',
        parameters=[{
            'use_sim_time': sim_time,
            'merge_count': ParameterValue(LaunchConfiguration('merge_count'), value_type=int)}],
        respawn=True,
        respawn_delay=2.0,
        output='screen')

    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laser_mapping',
        parameters=[config, {
            'use_sim_time': sim_time,
            'publish.map_en': map_on,
            'publish.scan_publish_en': map_on,
            'publish.path_en': map_on}],
        # It segfaults on every stop and would leave a core dump each time.
        prefix='prlimit --core=0',
        respawn=True,
        respawn_delay=2.0,
        output='screen')

    return LaunchDescription([use_sim_time, mapping, merge_count, merger, fast_lio])
