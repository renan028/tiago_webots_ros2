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
    lifecycle_nodes = ['amcl']

    # Getting directories and launch-files
    package_dir = get_package_share_directory('tiago_webots_ros2')

    # Amcl Node
    amcl_params = os.path.join(package_dir, 'config', 'amcl_params.yaml')
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params]
    )

    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])

    return LaunchDescription([
        amcl_node,
        lifecycle_manager
    ])
