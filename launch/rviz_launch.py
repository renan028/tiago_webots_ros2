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
    lifecycle_nodes = ['rviz2']

    # Getting directories and launch-files
    package_dir = get_package_share_directory('tiago_webots_ros2')

    # Rviz node
    rviz_config = os.path.join(package_dir, 'config', 'rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config]
    )

    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_rviz',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])

    return LaunchDescription([
        rviz_node,
        lifecycle_manager
    ])
