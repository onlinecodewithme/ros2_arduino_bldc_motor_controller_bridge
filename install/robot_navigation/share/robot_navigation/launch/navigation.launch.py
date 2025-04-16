#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the complete navigation stack with map server AMCL controller and planner.
    This file is the main entry point for starting the robot's autonomous navigation system.
    """
    # Get the package directory
    pkg_dir = get_package_share_directory('robot_navigation')
    
    # Set up paths
    map_yaml_path = os.path.join(pkg_dir, 'maps', 'simple_map.yaml')
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')
    
    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start lifecycle nodes')
    
    declare_map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_path,
        description='Full path to map YAML file')
    
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file')
    
    # Include robot state publisher launch file
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Include Nav2 Bringup launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file
        }.items()
    )
    
    # Launch map server separately to load our map
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
    
    # Launch lifecycle manager for map server
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
    
    # Launch AMCL for localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )
    
    # Launch our custom navigation node
    navigation_node = Node(
        package='robot_navigation',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Return the launch description
    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_autostart_arg,
        declare_map_yaml_arg,
        declare_params_file_arg,
        robot_state_publisher_launch,
        map_server_node,
        map_server_lifecycle_node,
        amcl_node,
        nav2_bringup_launch,
        navigation_node
    ])
