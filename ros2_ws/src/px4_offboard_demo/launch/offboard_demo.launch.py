from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_offboard_demo',
            executable='offboard_control',
            name='offboard_control',
            output='screen',
        ),
    ])

