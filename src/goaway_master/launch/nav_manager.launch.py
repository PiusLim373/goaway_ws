"""Navigation on saved maps (ported from tmms): localization_manager, navplan_processor, goaway_yasmin.

- localization_manager: /map_load and /relocalize (map from <maps_dir>/png/ + AMCL convergence),
  Lichtblick's /lichtblick_initialpose; publishes localization_status and /current_map.
- goaway_yasmin: the navigation state machine (/yasmin_state), Lichtblick's /lichtblick_navgoal,
  /cancel_goal, routes segment by segment with map switches.
- navplan_processor: /navigation_plan (the navplan tester, the C2) checked, cut into one segment
  per map, then /execute_route.

All three are order-independent (bounded waits) and need nav2 (bringup.launch.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    maps_dir = LaunchConfiguration('maps_dir')

    localization_manager = Node(
        package='localization_manager',
        executable='localization_manager_node',
        name='localization_manager',
        parameters=[{'maps_dir': maps_dir}],
        output='screen')

    # No name=: it would rename both nodes in this process (YasminNode, BasicNavigator) alike
    goaway_yasmin = Node(
        package='goaway_yasmin',
        executable='goaway_yasmin_node',
        output='screen')

    navplan_processor = Node(
        package='navplan_processor',
        executable='navplan_processor_node',
        name='navplan_processor',
        parameters=[{'maps_dir': maps_dir}],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('maps_dir', default_value='~/.htxgrrt/maps'),
        localization_manager, goaway_yasmin, navplan_processor])
