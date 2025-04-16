#!/usr/bin/env python3

"""
Launch file for complete navigation stack with auto-detecting Arduino bridge
that automatically finds available Arduino ports and handles permission issues.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get directories
    arduino_pkg_dir = get_package_share_directory('arduino_motor_bridge')
    robot_nav_pkg_dir = get_package_share_directory('robot_navigation')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(robot_nav_pkg_dir, 'maps', 'simple_map.yaml'),
        description='Path to map file to use'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the navigation stack'
    )
    
    # Include the auto-detecting Arduino bridge launch
    arduino_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arduino_pkg_dir, 'launch', 'auto_arduino_bridge.launch.py')
        ),
        # You can override any parameters from the included launch file here
        # launch_arguments={
        #    'serial_port': 'auto'
        # }.items()
    )
    
    # Include the nav2 stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 
                        'launch', 
                        'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(robot_nav_pkg_dir, 'config', 'nav2_params.yaml'),
            'use_sim_time': use_sim_time,
            'autostart': autostart
        }.items()
    )
    
    # Robot state publisher node
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_pkg_dir, 'launch', 'robot_state_publisher.launch.py')
        )
    )
    
    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }]
    )
    
    # Lifecycle manager for map server
    map_server_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server']
        }]
    )
    
    # Create and return launch description
    ld = LaunchDescription()
    
    # Add declarations
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_autostart)
    
    # Add launches
    ld.add_action(arduino_bridge_launch)
    ld.add_action(robot_state_publisher_launch)
    ld.add_action(map_server_node)
    ld.add_action(map_server_lifecycle_node)
    ld.add_action(nav2_launch)
    
    return ld
