#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch navigation with complex map.
    
    This launch file integrates:
    1. The robot state publisher for TF frames
    2. The Nav2 navigation stack with complex map server
    3. The auto-detecting Arduino motor bridge for hardware control
    4. RViz for visualization
    """
    # Get package directories
    nav_pkg_dir = get_package_share_directory('robot_navigation')
    arduino_pkg_dir = get_package_share_directory('arduino_motor_bridge')
    
    # Define the map file paths
    map_file = os.path.join(nav_pkg_dir, 'maps', 'complex_map.yaml')
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml_file = LaunchConfiguration('map')
    
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')
    
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz if true')
        
    declare_map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map yaml file')
    
    # Include robot state publisher launch
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg_dir, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include auto-detecting Arduino bridge launch
    arduino_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(arduino_pkg_dir, 'launch', 'auto_arduino_bridge.launch.py')
        ])
    )
    
    # Include navigation launch with map server, AMCL, planners, etc.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg_dir, 'launch', 'navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file
        }.items()
    )
    
    # Create path to custom rviz config
    rviz_config_dir = os.path.join(nav_pkg_dir, 'config')
    rviz_config_file = os.path.join(rviz_config_dir, 'nav2_visualization.rviz')
            
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_use_rviz_arg,
        declare_map_yaml_file,
        robot_state_publisher_launch,
        arduino_bridge_launch,
        navigation_launch,
        rviz_node
    ])
