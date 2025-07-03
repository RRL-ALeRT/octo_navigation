import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # path to this pkg
    pkg_bring_up = get_package_share_directory("bring_up_alert_nav")

    # Launch arguments

    # Move Base Flex
    move_base_flex = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_bring_up, "launch", "mbf_alert_nav_server_launch.py"]
            )
        ),
    )

    # Rviz2 send_goal Node
    delayed_exe_path = Node(
        package="bring_up_alert_nav",
        executable="exe_path_node",
        name="exe_path",
        output="screen")
    
    # Node for octo_planner
    posture_mgr = Node(
        package="bring_up_alert_nav",
        executable='posture_manager',
        parameters=[{
            'squat_path_topic': '/move_base_flex/Squatpath',
            'odom_topic': '/Spot/odometry',
            'height_service': '/Spot/set_height'
        }]
    )

    map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    return LaunchDescription(

        [
            #map_odom,
            move_base_flex,
            delayed_exe_path,
            posture_mgr,
        ]
    )