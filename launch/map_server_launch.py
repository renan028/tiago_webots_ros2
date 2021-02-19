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
    lifecycle_nodes = ['map_server']

    # Getting directories and launch-files
    package_dir = get_package_share_directory('tiago_webots_ros2')

    # Map Server Node
    map_file = os.path.join(package_dir, 'resources', 'map', 'intralogistics.yaml')
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file,
            'topic_name': 'map',
            'frame_id': 'map'}]
    )

    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])

    return LaunchDescription([
        map_server_node,
        lifecycle_manager
    ])
