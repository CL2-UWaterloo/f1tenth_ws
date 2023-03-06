from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'sim_config.yaml'
        )

    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        parameters=[config]
    )
    
    waypoint_visualizer_node = Node(
        package='pure_pursuit',
        executable='waypoint_visualiser_node',
        name='waypoint_visualiser_node',
        parameters=[config]
    )

    # finalize
    ld.add_action(pure_pursuit_node)
    ld.add_action(waypoint_visualizer_node)

    return ld