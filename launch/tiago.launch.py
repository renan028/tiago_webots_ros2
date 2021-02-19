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

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Map Server node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(map_server_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                  lifecycle_node_matcher=matches_node_name(node_name='map_server'),
                  transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN
                )),
                LogInfo(msg="[LifecycleLaunch] Map Server node is exiting.")
            ]
        )
    )

    # Rviz node
    use_rviz = launch.substitutions.LaunchConfiguration('rviz', default=True)
    rviz_config = os.path.join(get_package_share_directory(package_name), 'config', 'rviz.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    return launch.LaunchDescription([
        webots,
        tiago_robot_node,
        map_server_node,
        rviz
    ])