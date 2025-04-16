from setuptools import setup
import os
from glob import glob

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
        # Include map files
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*'))),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='x4',
    maintainer_email='onlinecodewithme@gmail.com',
    description='ROS2 Navigation package for Arduino-controlled robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = robot_navigation.navigation_node:main',
            'navigation_cli = robot_navigation.navigation_cli:main',
            'test_goal_navigation = robot_navigation.test_goal_navigation:main',
            'simulated_odometry = robot_navigation.simulated_odometry:main',
        ],
    },
)
