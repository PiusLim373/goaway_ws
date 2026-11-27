import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
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
    # legs: odom from the legs + IMU only; fused: FAST_LIO pulls it into place
    odom_source_arg = DeclareLaunchArgument('odom_source', default_value='legs')
    nav2_arg = DeclareLaunchArgument('nav2', default_value='true')
    # Not 'mapping': the FAST_LIO include sets that launch configuration for the whole launch
    mapping_arg = DeclareLaunchArgument('slam_mapping', default_value='true')
    maps_dir_arg = DeclareLaunchArgument('maps_dir', default_value='~/.htxgrrt/maps')

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
        parameters=[{'odom_source': LaunchConfiguration('odom_source')}],
        output='screen')

    # Height scan for the walk policy (/height_scan), with markers for Lichtblick
    height_map = Node(
        package='goaway_locomotion',
        executable='height_map_node',
        parameters=[os.path.join(
            get_package_share_directory('goaway_master'), 'config', 'goaway_height_map.yaml')],
        condition=IfCondition(LaunchConfiguration('height_map')),
        output='screen')

    # /utlidar/cloud without the robot's body and legs -> /utlidar/cloud_filtered (costmap)
    lidar_self_filter = Node(
        package='pcl_ros',
        executable='filter_crop_box_node',
        name='lidar_self_filter',
        parameters=[os.path.join(
            get_package_share_directory('goaway_master'), 'config', 'goaway_lidar_filter.yaml')],
        remappings=[('input', '/utlidar/cloud'), ('output', '/utlidar/cloud_filtered')],
        respawn=True,
        respawn_delay=2.0,
        output='screen')

    # /utlidar/cloud_filtered stacked in odom -> /utlidar/scan (AMCL) + /utlidar/cloud_accumulated
    cloud_accumulator = Node(
        package='mapping_utils',
        executable='cloud_accumulator_node',
        name='cloud_accumulator',
        parameters=[os.path.join(
            get_package_share_directory('goaway_master'), 'config', 'goaway_scan.yaml')],
        respawn=True,
        respawn_delay=2.0,
        output='screen')

    # nav2 (bringup -> localization + navigation): the UI's safety teleop (assisted_teleop on the
    # local costmap), AMCL on /utlidar/scan, goals from Lichtblick. Blank startup map until mapping.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('goaway_master'), 'launch', 'bringup.launch.py')),
        condition=IfCondition(LaunchConfiguration('nav2')))

    # Navigation on saved maps (tmms): localization_manager, goaway_yasmin, navplan_processor
    nav_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('goaway_master'), 'launch', 'nav_manager.launch.py')),
        launch_arguments={'maps_dir': LaunchConfiguration('maps_dir')}.items(),
        condition=IfCondition(LaunchConfiguration('nav2')))

    # Mapping sessions (slam_toolbox, idle until ~/start_mapping): maps in ~/.htxgrrt/maps
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('goaway_master'), 'launch', 'mapping.launch.py')),
        launch_arguments={'maps_dir': LaunchConfiguration('maps_dir')}.items(),
        condition=IfCondition(LaunchConfiguration('slam_mapping')))

    return LaunchDescription([
        log_dir, fastlio_mapping, height_map_arg, odom_source_arg, rosbridge, robot_state_publisher,
        quadruped_controller, locomotion, fastlio, odometry, height_map, lidar_self_filter,
        cloud_accumulator, nav2_arg, nav2, nav_manager,
        mapping_arg, maps_dir_arg, mapping])
