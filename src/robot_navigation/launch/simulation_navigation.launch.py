#!/usr/bin/env python3

"""
Launch file for simulated robot navigation with virtual odometry.

This launch file:
1. Launches a simulated odometry publisher (no physical encoders needed)
2. Loads the map server
3. Starts the Nav2 navigation stack
4. Connects to Arduino for motor control (real motor commands)
5. Opens RViz for visualization

This is ideal for visualizing navigation without real odometry feedback.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    # Get paths to packages
    nav_pkg_dir = get_package_share_directory('robot_navigation')
    arduino_pkg_dir = get_package_share_directory('arduino_motor_bridge')
    
    # Get parameters
    map_yaml = os.path.join(nav_pkg_dir, 'maps', 'simple_map.yaml')
    nav_params = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(nav_pkg_dir, 'config', 'nav2_visualization.rviz')
    
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    serial_port = LaunchConfiguration('serial_port')
    auto_port_detect = LaunchConfiguration('auto_port_detect')
    
    # Declare parameters
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Use RViz for visualization',
    )
    
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM1',
        description='Serial port for Arduino',
    )
    
    declare_auto_port_detect = DeclareLaunchArgument(
        'auto_port_detect',
        default_value='true',
        description='Auto-detect Arduino port',
    )
    
    # Use find_arduino_port.py to auto-detect Arduino port
    arduino_port_finder = Node(
        package='arduino_motor_bridge',
        executable='test_arduino_commands',
        name='arduino_port_finder',
        output='screen',
        arguments=['--find-port-only'],
        condition=IfCondition(auto_port_detect),
    )
    
    # Define robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': False, 
             'publish_frequency': 15.0}
        ],
        remappings=[],
    )
    
    # Static transform publisher from map to odom
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    
    # Arduino bridge node for motor control
    arduino_bridge = Node(
        package='arduino_motor_bridge',
        executable='arduino_bridge_node',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': 115200,
            'wheel_radius': 0.065,
            'base_width': 0.17,
            'max_linear_speed': 0.5,
            'max_angular_speed': 2.0,
        }],
        emulate_tty=True
    )
    
    # Simulated odometry publisher - NEW!
    simulated_odometry = Node(
        package='robot_navigation',
        executable='simulated_odometry',
        name='simulated_odometry',
        output='screen',
        parameters=[{
            'wheel_radius': 0.065,
            'base_width': 0.17,
            'publish_rate': 20.0,
            'max_linear_speed': 0.5,
            'max_angular_speed': 2.0,
            'acceleration': 0.5,
            'angular_acceleration': 1.0,
        }],
        emulate_tty=True
    )
    
    # Launch navigation stack (Nav2)
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
    )
    
    # Launch Nav2 with complete functionalities
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav_params,
            'use_sim_time': 'false',
        }.items(),
    )
    
    # Launch RViz only if requested
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )
    
    # Create navigation node to handle goals
    navigation_node = Node(
        package='robot_navigation',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add declared arguments
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_serial_port)
    ld.add_action(declare_auto_port_detect)
    
    # Add Arduino port finder
    ld.add_action(arduino_port_finder)
    
    # Add nodes and includes in launch order
    ld.add_action(robot_state_publisher)
    ld.add_action(tf_map_to_odom)
    ld.add_action(robot_state_publisher_launch)
    ld.add_action(arduino_bridge)
    ld.add_action(simulated_odometry)  # Added the simulated odometry node
    ld.add_action(nav2_launch)
    ld.add_action(rviz_node)
    ld.add_action(navigation_node)
    
    return ld
