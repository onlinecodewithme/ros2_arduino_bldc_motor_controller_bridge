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
    Launch complex map navigation with E38S encoder integration.
    
    This launch file combines the complex map navigation with high-resolution
    encoder odometry to enable accurate robot positioning and visualization.
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
    
    # Encoder-specific parameters
    serial_port = LaunchConfiguration('serial_port')
    wheel_radius = LaunchConfiguration('wheel_radius')
    base_width = LaunchConfiguration('base_width')
    ticks_per_rev = LaunchConfiguration('ticks_per_rev')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    publish_rate = LaunchConfiguration('publish_rate')
    
    # Declare launch arguments - basic
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
    
    # Declare launch arguments - encoder specific
    declare_serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino communication')
        
    declare_wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0825',
        description='Wheel radius in meters')
        
    declare_base_width_arg = DeclareLaunchArgument(
        'base_width',
        default_value='0.17',
        description='Distance between wheels in meters')
        
    declare_ticks_per_rev_arg = DeclareLaunchArgument(
        'ticks_per_rev',
        default_value='4000.0',
        description='Encoder ticks per revolution (4000 for E38S with quadrature)')
        
    declare_max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.5',
        description='Maximum linear speed in m/s')
        
    declare_max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='2.0',
        description='Maximum angular speed in rad/s')
        
    declare_publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='20.0',
        description='Odometry publish rate in Hz')
    
    # Include robot state publisher launch
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg_dir, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include robust encoder bridge launch instead of auto arduino bridge
    encoder_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(arduino_pkg_dir, 'launch', 'robust_encoder_bridge.launch.py')
        ]),
        launch_arguments={
            'serial_port': serial_port,
            'wheel_radius': wheel_radius,
            'base_width': base_width,
            'encoder_ticks_per_rev': ticks_per_rev,
            'max_linear_speed': max_linear_speed,
            'max_angular_speed': max_angular_speed,
            'publish_rate': publish_rate
        }.items()
    )
    
    # Include navigation launch with map server, AMCL, planners, etc.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg_dir, 'launch', 'navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'encoder_odometry': 'true',
            'serial_port': serial_port,
            'wheel_radius': wheel_radius,
            'base_width': base_width,
            'ticks_per_rev': ticks_per_rev,
            'max_linear_speed': max_linear_speed,
            'max_angular_speed': max_angular_speed,
            'publish_rate': publish_rate
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
        # Basic args
        declare_use_sim_time_arg,
        declare_use_rviz_arg,
        declare_map_yaml_file,
        
        # Encoder-specific args
        declare_serial_port_arg,
        declare_wheel_radius_arg,
        declare_base_width_arg,
        declare_ticks_per_rev_arg,
        declare_max_linear_speed_arg,
        declare_max_angular_speed_arg,
        declare_publish_rate_arg,
        
        # Nodes and launches
        robot_state_publisher_launch,
        encoder_bridge_launch,
        navigation_launch,
        rviz_node
    ])
