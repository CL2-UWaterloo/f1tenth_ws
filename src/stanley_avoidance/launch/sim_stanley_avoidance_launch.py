from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('stanley_avoidance'),
        'config',
        'sim_config.yaml'
    )

    stanley_avoidance_node = Node(
        package='stanley_avoidance',
        executable='stanley_avoidance',
        name='ego_stanley_avoidance',
        parameters=[config]
    )

    waypoint_visualizer_node = Node(
        package='pure_pursuit',
        executable='waypoint_visualizer',
        name='waypoint_visualizer_node',
        parameters=[config]
    )

    # finalize
    ld.add_action(stanley_avoidance_node)
    ld.add_action(waypoint_visualizer_node)

    return ld
