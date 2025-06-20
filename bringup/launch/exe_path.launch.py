from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bringup',
            executable='exe_path_node',
            name='exe_path_node'
        )
    ])
