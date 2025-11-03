from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(os.path.dirname(__file__), '..', 'config', 'pid_circle.yaml'),
            description='YAML with PID and mission params',
        ),
        Node(
            package='px4_offboard_demo',
            executable='offboard_pid',
            name='px4_offboard_demo',
            output='screen',
            parameters=[params_file],
        ),
    ])

