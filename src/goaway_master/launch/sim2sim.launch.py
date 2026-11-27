import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Sim2Sim with unitree_rl_mjlab's simulator on lo (simulate/config.yaml l1_cloud: 1): the robot's
# stack without FAST_LIO. The simulator's L1 cloud -> goaway_odometry (legs + IMU) ->
# height_map_node (instant) -> /height_scan -> goaway_locomotion_node.
def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('goaway_description'), 'urdf', 'go2.urdf')
    with open(urdf_path) as f:
        robot_description = f.read()

    height_map_arg = DeclareLaunchArgument('height_map', default_value='true')
    rosbridge_arg = DeclareLaunchArgument('rosbridge', default_value='true')
    odom_source_arg = DeclareLaunchArgument('odom_source', default_value='legs')
    nav2_arg = DeclareLaunchArgument('nav2', default_value='true')
    # Not 'mapping': the FAST_LIO include sets that launch configuration for the whole launch
    mapping_arg = DeclareLaunchArgument('slam_mapping', default_value='true')
    maps_dir_arg = DeclareLaunchArgument('maps_dir', default_value='~/.htxgrrt/maps')

    # For goaway_ui / Lichtblick
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch', 'rosbridge_websocket_launch.xml')),
        condition=IfCondition(LaunchConfiguration('rosbridge')))

    # base_link -> utlidar_lidar for the height map
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen')

    quadruped_controller = Node(
        package='goaway_quadruped_controller',
        executable='goaway_quadruped_controller_node',
        output='screen')

    locomotion = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('goaway_locomotion'),
                          'lib', 'goaway_locomotion', 'goaway_locomotion_node'),
             '--network', 'lo'],
        name='goaway_locomotion_node',
        output='screen')

    # The sim's feet don't slip: leg speeds as measured.
    odometry = Node(
        package='goaway_odometry',
        executable='goaway_odometry_node',
        parameters=[{'scale_forward': 1.0, 'scale_side': 1.0,
                     'odom_source': LaunchConfiguration('odom_source')}],
        output='screen')

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
        output='screen')

    # /utlidar/cloud_filtered stacked in odom -> /utlidar/scan (AMCL) + /utlidar/cloud_accumulated
    cloud_accumulator = Node(
        package='mapping_utils',
        executable='cloud_accumulator_node',
        name='cloud_accumulator',
        parameters=[os.path.join(
            get_package_share_directory('goaway_master'), 'config', 'goaway_scan.yaml')],
        output='screen')

    # nav2 (bringup -> localization + navigation): the UI's safety teleop (assisted_teleop on the
    # local costmap), AMCL on /utlidar/scan, goals from Lichtblick. Blank startup map until mapping.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('goaway_master'), 'launch', 'bringup.launch.py')),
        condition=IfCondition(LaunchConfiguration('nav2')))

    # Navigation on saved maps (tmms), with nav2.
    # localization_manager: /map_load, /relocalize, Lichtblick's /lichtblick_initialpose
    localization_manager = Node(
        package='localization_manager',
        executable='localization_manager_node',
        name='localization_manager',
        parameters=[{'maps_dir': LaunchConfiguration('maps_dir')}],
        condition=IfCondition(LaunchConfiguration('nav2')),
        output='screen')

    # goaway_yasmin: the navigation state machine, Lichtblick's /lichtblick_navgoal, /cancel_goal.
    # No name=: it would rename both nodes in its process (YasminNode, BasicNavigator) alike.
    goaway_yasmin = Node(
        package='goaway_yasmin',
        executable='goaway_yasmin_node',
        condition=IfCondition(LaunchConfiguration('nav2')),
        output='screen')

    # navplan_processor: /navigation_plan checked, cut into one segment per map, then /execute_route
    navplan_processor = Node(
        package='navplan_processor',
        executable='navplan_processor_node',
        name='navplan_processor',
        parameters=[{'maps_dir': LaunchConfiguration('maps_dir')}],
        condition=IfCondition(LaunchConfiguration('nav2')),
        output='screen')

    # Mapping sessions (slam_toolbox, idle until ~/start_mapping): maps in ~/.htxgrrt/maps
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('goaway_master'), 'launch', 'mapping.launch.py')),
        launch_arguments={'maps_dir': LaunchConfiguration('maps_dir')}.items(),
        condition=IfCondition(LaunchConfiguration('slam_mapping')))

    return LaunchDescription([
        height_map_arg, rosbridge_arg, odom_source_arg, rosbridge, robot_state_publisher, quadruped_controller,
        locomotion, odometry, height_map, lidar_self_filter, cloud_accumulator, nav2_arg, nav2,
        localization_manager, goaway_yasmin, navplan_processor,
        mapping_arg, maps_dir_arg, mapping])
