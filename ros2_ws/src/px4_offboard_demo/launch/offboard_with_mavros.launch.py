from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mavros_share = get_package_share_directory('mavros')
    container_launch = os.path.join(mavros_share, 'launch', 'mavros_container.launch.py')

    if os.path.exists(container_launch):
        mavros_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(container_launch),
            launch_arguments={
                'fcu_url': 'udp://:14540@127.0.0.1:14557',
            }.items(),
        )
    else:
        # Fallback to px4.launch in binary packages
        px4_launch = os.path.join(mavros_share, 'launch', 'px4.launch')
        pluginlists_yaml = os.path.join(mavros_share, 'launch', 'px4_pluginlists.yaml')
        config_yaml = os.path.join(mavros_share, 'launch', 'px4_config.yaml')
        mavros_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(px4_launch),
            launch_arguments={
                'fcu_url': 'udp://:14540@127.0.0.1:14557',
                'pluginlists_yaml': pluginlists_yaml,
                'config_yaml': config_yaml,
            }.items(),
        )

    offboard = Node(
        package='px4_offboard_demo',
        executable='offboard_control',
        name='offboard_control',
        output='screen',
    )

    return LaunchDescription([
        mavros_launch,
        offboard,
    ])
