from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ta_hdi_fmp_planner',
            executable='planner_node',
            output='screen',
            parameters=[{'frame_id': 'map'}],
        )
    ])
