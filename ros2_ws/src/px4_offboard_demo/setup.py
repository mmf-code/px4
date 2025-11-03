from setuptools import setup

package_name = 'px4_offboard_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/offboard_demo.launch.py',
            'launch/offboard_with_mavros.launch.py',
            'launch/pid_mission.launch.py',
            'launch/viz.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/pid_hover.yaml',
            'config/pid_circle.yaml',
            'config/waypoints_demo.yaml',
            'config/zigzag.yaml',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/offboard_view.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Demo',
    maintainer_email='you@example.com',
    description='PX4 offboard control demo using MAVROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = px4_offboard_demo.offboard_control:main',
            'offboard_pid = px4_offboard_demo.controller_pid:main',
        ],
    },
)
