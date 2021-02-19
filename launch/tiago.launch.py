#!/usr/bin/env python

"""Launch Webots and the controller."""

import os

import launch
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

package_name = 'tiago_webots_ros2'

def generate_launch_description():
    package_dir = get_package_share_directory(package_name)

    # Webots
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments={
            'executable': 'webots_differential_drive_node',
            'world': os.path.join(package_dir, 'worlds', 'intralogistics_2.wbt'),
            'node_parameters': os.path.join(package_dir, 'config', 'tiago.yaml'),
            'output': 'screen'
        }.items()
    )

    # TiagoRobot node
    tiago_params = os.path.join(package_dir, 'config', 'tiago_params.yaml')
    tiago_robot_node = Node(
        package='tiago_webots_ros2',
        executable='robot_task_node',
        output='screen',
        parameters=[tiago_params]
    )

    # Map Server Node
    map_server_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tiago_webots_ros2'), 'launch', 'map_server_launch.py')
        ),
        launch_arguments={}.items()
    )

    # Amcl Node
    amcl_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tiago_webots_ros2'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={}.items()
    )

    # Navigator Node
    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tiago_webots_ros2'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={}.items()
    )

    # Rviz node
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tiago_webots_ros2'), 'launch', 'rviz_launch.py')
        ),
        launch_arguments={}.items()
    )

    return launch.LaunchDescription([
        webots,
        tiago_robot_node,
        map_server_node,
        amcl_node,
        navigation_node,
        rviz_node
    ])