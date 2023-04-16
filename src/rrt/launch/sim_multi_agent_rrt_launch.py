from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('rrt'),
        'config',
        'sim_config.yaml'
        )

    rrt_node = Node(
        package='rrt',
        executable='rrt',
        name='ego_rrt',
        parameters=[config]
    )
    
    rrt_node_opp = Node(
        package='rrt',
        executable='rrt',
        name='opp_rrt',
        parameters=[config]
    )

    waypoint_visualizer_node = Node(
        package='pure_pursuit',
        executable='waypoint_visualiser_node',
        name='waypoint_visualiser_node',
        parameters=[config]
    )

    # finalize
    ld.add_action(rrt_node)
    ld.add_action(rrt_node_opp)
    ld.add_action(waypoint_visualizer_node)

    return ld