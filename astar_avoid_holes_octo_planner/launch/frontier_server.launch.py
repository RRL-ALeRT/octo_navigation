from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='astar_avoid_holes_octo_planner',
            executable='frontier_server',
            name='frontier_server',
            output='screen',
            parameters=[{
                # Topic auf dem octomap_server seine Karte published
                'octomap_topic':     '/octomap_binary',

                # TF-Frames
                'robot_frame':       'base_link',
                'map_frame':         'map',

                # Maximale Distanz zwischen zwei Frontier-Voxeln
                # damit sie zum gleichen Cluster gehören (Meter)
                'cluster_radius':    0.5,

                # Wie weit vor der Frontier-Grenze das Navigationsziel
                # platziert wird (Meter). Spot bleibt damit sicher im
                # bekannten freien Raum.
                'standoff_distance': 1.0,
            }],
        ),
    ])
