#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the complete navigation stack with map server, AMCL, controller, and planner.
    This file is the main entry point for starting the robot's autonomous navigation system.
    """
    # Get the package directories
    nav_pkg_dir = get_package_share_directory('robot_navigation')
    arduino_pkg_dir = get_package_share_directory('arduino_motor_bridge')

    # Set up paths
    map_yaml_path = os.path.join(nav_pkg_dir, 'maps', 'simple_map.yaml')
    nav2_params_path = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    # E38S encoder-specific arguments
    encoder_odometry = LaunchConfiguration('encoder_odometry')
    serial_port = LaunchConfiguration('serial_port')
    wheel_radius = LaunchConfiguration('wheel_radius')
    base_width = LaunchConfiguration('base_width')
    ticks_per_rev = LaunchConfiguration('ticks_per_rev')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    publish_rate = LaunchConfiguration('publish_rate')

    # Declare basic launch arguments
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
        
    # Declare encoder-specific launch arguments
    declare_encoder_odometry_arg = DeclareLaunchArgument(
        'encoder_odometry',
        default_value='false',
        description='Use E38S high-resolution encoders for odometry')
        
    declare_serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino connection')
        
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

    # Include robot state publisher launch file
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg_dir, 'launch', 'robot_state_publisher.launch.py')
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
    
    # Launch E38S encoder odometry node when encoder_odometry is true
    encoder_odometry_node = Node(
        condition=IfCondition(encoder_odometry),
        package='arduino_motor_bridge',
        executable='encoder_odometry_node',
        name='encoder_odometry_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'wheel_radius': wheel_radius,
            'base_width': base_width,
            'encoder_ticks_per_rev': ticks_per_rev,
            'max_linear_speed': max_linear_speed,
            'max_angular_speed': max_angular_speed,
            'publish_rate': publish_rate,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'cmd_vel_topic': 'cmd_vel',
            'odom_topic': 'odom'
        }],
        emulate_tty=True
    )
    
    # Launch standard improved arduino bridge when encoder_odometry is false
    standard_arduino_bridge = Node(
        condition=UnlessCondition(encoder_odometry),
        package='arduino_motor_bridge',
        executable='improved_arduino_bridge',
        name='improved_arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'wheel_radius': wheel_radius,
            'base_width': base_width
        }],
        emulate_tty=True
    )

    # Return the launch description
    return LaunchDescription([
        # Basic launch arguments
        declare_use_sim_time_arg,
        declare_autostart_arg,
        declare_map_yaml_arg,
        declare_params_file_arg,
        
        # Encoder-specific launch arguments
        declare_encoder_odometry_arg,
        declare_serial_port_arg,
        declare_wheel_radius_arg,
        declare_base_width_arg,
        declare_ticks_per_rev_arg,
        declare_max_linear_speed_arg,
        declare_max_angular_speed_arg,
        declare_publish_rate_arg,
        
        # Nodes and includes
        robot_state_publisher_launch,
        map_server_node,
        map_server_lifecycle_node,
        amcl_node,
        nav2_bringup_launch,
        navigation_node,
        
        # Bridge nodes (mutually exclusive based on encoder_odometry)
        encoder_odometry_node,
        standard_arduino_bridge
    ])
