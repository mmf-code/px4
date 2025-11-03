from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_pid = LaunchConfiguration('use_pid')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    default_params = os.path.join(os.path.dirname(__file__), '..', 'config', 'pid_circle.yaml')
    default_rviz = os.path.join(os.path.dirname(__file__), '..', 'rviz', 'offboard_view.rviz')

    nodes = [
        DeclareLaunchArgument('use_pid', default_value='true', description='Run PID offboard controller'),
        DeclareLaunchArgument('params_file', default_value=default_params, description='PID params'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz, description='RViz config path'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            additional_env={
                'QT_QPA_PLATFORM': 'xcb',
                'LD_LIBRARY_PATH': ''
            },
            output='screen',
        ),
    ]

    # Optionally run PID node
    nodes.append(
        Node(
            package='px4_offboard_demo',
            executable='offboard_pid',
            name='px4_offboard_demo',
            output='screen',
            parameters=[params_file],
            condition=None,
        )
    )

    return LaunchDescription(nodes)
