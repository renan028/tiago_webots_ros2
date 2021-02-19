import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Variables
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    # Getting directories and launch-files
    package_dir = get_package_share_directory('tiago_webots_ros2')

    # Controller Node
    controller_params = os.path.join(package_dir, 'config', 'controller_server_params.yaml')
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_params]
    )

    # Planner Node
    planner_params = os.path.join(package_dir, 'config', 'planner_server_params.yaml')
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_params]
    )

    # Recovery Node
    recovery_params = os.path.join(package_dir, 'config', 'recovery_server_params.yaml')
    recovery_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[recovery_params]
    )

    # BT Node
    bt_params = os.path.join(package_dir, 'config', 'bt_navigator_params.yaml')
    bt_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_params]
    )

    # Waypoints Node
    waypoint_params = os.path.join(package_dir, 'config', 'waypoint_follower_params.yaml')
    waypoint_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[waypoint_params]
    )

    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])

    return LaunchDescription([
        controller_node,
        planner_node,
        recovery_node,
        bt_node,
        waypoint_node,
        lifecycle_manager
    ])
