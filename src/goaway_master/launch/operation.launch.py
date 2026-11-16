import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('goaway_description'), 'urdf', 'go2.urdf')
    with open(urdf_path) as f:
        robot_description = f.read()

    log_dir = DeclareLaunchArgument(
        'locomotion_log_dir', default_value='/home/htxgrrt/.htxgrrt/logs/goaway_locomotion')
    fastlio_mapping = DeclareLaunchArgument('fastlio_mapping', default_value='false')
    height_map_arg = DeclareLaunchArgument('height_map', default_value='true')

    # websocket on port 9090 for Foxglove / web UIs (also starts rosapi). respawn (both nodes)
    # is what lets the reboot manager restart a wedged rosbridge with a SIGINT alone.
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch', 'rosbridge_websocket_launch.xml')),
        launch_arguments={'respawn': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen')

    # /lf/lowstate -> /joint_states
    quadruped_controller = Node(
        package='goaway_quadruped_controller',
        executable='goaway_quadruped_controller_node',
        output='screen')

    # Locomotion FSM: an SDK program, not a ROS node (ExecuteProcess: it rejects --ros-args).
    # Starts in Off (no motor commands); damps before exiting on shutdown.
    locomotion = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('goaway_locomotion'),
                          'lib', 'goaway_locomotion', 'goaway_locomotion_node'),
             '--network', EnvironmentVariable('GOAWAY_IFACE', default_value=''),
             '--log', '--log_dir', LaunchConfiguration('locomotion_log_dir')],
        name='goaway_locomotion_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0)

    # Always on, so it is ready when Unitree's odom stops (HTX).
    fastlio = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('goaway_master'), 'launch', 'fastlio_mapping.launch.py')),
        launch_arguments={'mapping': LaunchConfiguration('fastlio_mapping')}.items())

    # odom -> base_footprint -> base_link, and odom -> camera_init (FAST_LIO's frame)
    odometry = Node(
        package='goaway_odometry',
        executable='goaway_odometry_node',
        output='screen')

    # Height scan for the walk policy (/height_scan), with markers for Lichtblick
    height_map = Node(
        package='goaway_locomotion',
        executable='height_map_node',
        parameters=[os.path.join(
            get_package_share_directory('goaway_master'), 'config', 'goaway_height_map.yaml')],
        condition=IfCondition(LaunchConfiguration('height_map')),
        output='screen')

    return LaunchDescription([
        log_dir, fastlio_mapping, height_map_arg, rosbridge, robot_state_publisher,
        quadruped_controller, locomotion, fastlio, odometry, height_map])
