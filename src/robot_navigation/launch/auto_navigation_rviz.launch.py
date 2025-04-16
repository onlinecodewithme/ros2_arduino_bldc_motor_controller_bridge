#!/usr/bin/env python3
"""
Launch file for running navigation with RViz visualization using the auto-detecting Arduino bridge.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get directories
    arduino_pkg_dir = get_package_share_directory('arduino_motor_bridge')
    robot_nav_pkg_dir = get_package_share_directory('robot_navigation')
    
    # Get paths to files
    rviz_config_file = os.path.join(robot_nav_pkg_dir, 'config', 'nav2_visualization.rviz')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # Include the auto-detecting Arduino bridge launch
    arduino_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arduino_pkg_dir, 'launch', 'auto_arduino_bridge.launch.py')
        )
    )
    
    # Include the navigation_rviz.launch.py but skip the Arduino bridge
    nav_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_pkg_dir, 'launch', 'navigation_rviz.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # Skip launching the original Arduino bridge
            'autostart': 'true'
        }.items()
    )
    
    # Create and return launch description
    ld = LaunchDescription()
    
    # Add declaration
    ld.add_action(declare_use_sim_time)
    
    # Add our auto-detecting Arduino bridge
    ld.add_action(arduino_bridge_launch)
    
    # Add navigation with RViz visualization
    ld.add_action(nav_rviz_launch)
    
    return ld
