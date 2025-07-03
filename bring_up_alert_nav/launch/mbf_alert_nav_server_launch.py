import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    mbf_octo_nav_config = os.path.join(
        get_package_share_directory("bring_up_alert_nav"), "params", "mbf_alert_nav.yaml"
    )

    octo_nav_server = Node(
        name="move_base_flex",
        package="mbf_octo_nav",
        executable="mbf_octo_nav",
        remappings=[
            ("/move_base_flex/cmd_vel", "/cmd_vel"),
        ],
        parameters=[
            mbf_octo_nav_config,
        ],
    )

    return LaunchDescription(
        [
            octo_nav_server,
        ]
    )