from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'drone_control_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Team',
    maintainer_email='your-email@example.com',
    description='ROS2 Python algorithms for autonomous drone control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_controller = drone_control_pkg.flight_control.flight_controller_node:main',
            'safety_monitor = drone_control_pkg.flight_control.safety_monitor_node:main',
            'waypoint_navigator = drone_control_pkg.navigation.waypoint_navigator_node:main',
            'path_planner = drone_control_pkg.navigation.path_planner_node:main',
            'mavlink_bridge = drone_control_pkg.communication.mavlink_bridge_node:main',
            'telemetry_node = drone_control_pkg.communication.telemetry_node:main',
        ],
    },
)
