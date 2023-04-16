from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('rrt'),
        'config',
        'config.yaml'
        )

    rrt_node = Node(
        package='rrt',
        executable='rrt',
        name='rrt',
        parameters=[config]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('rrt'), 'launch', 'rrt.rviz')]
    )

    waypoint_visualizer_node = Node(
        package='pure_pursuit',
        executable='waypoint_visualiser_node',
        name='waypoint_visualiser_node',
        parameters=[config]
    )


    # finalize
    ld.add_action(rviz_node)
    ld.add_action(rrt_node)
    ld.add_action(waypoint_visualizer_node)

    return ld