from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ta_hdi_fmp_mobile_adapter',
            executable='mobile_adapter_node',
            output='screen',
            parameters=[{
                'lookahead_dist': 0.6,
                'v_max': 0.25,
                'w_max': 1.0,
                'goal_tolerance': 0.15,
                'timeout_sec': 5.0,
            }],
        )
    ])
