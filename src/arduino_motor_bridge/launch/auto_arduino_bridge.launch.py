#!/usr/bin/env python3

"""
Launch file for the auto-detecting Arduino bridge node that automatically finds
available Arduino ports and handles permission issues.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('arduino_motor_bridge')
    
    # Launch arguments
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    wheel_radius = LaunchConfiguration('wheel_radius')
    base_width = LaunchConfiguration('base_width')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    connection_retry_seconds = LaunchConfiguration('connection_retry_seconds')
    
    # Declare launch arguments
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='auto',
        description='Serial port for Arduino connection (auto for automatic detection)'
    )
    
    declare_baud_rate = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    declare_wheel_radius = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.065',
        description='Wheel radius in meters'
    )
    
    declare_base_width = DeclareLaunchArgument(
        'base_width',
        default_value='0.17',
        description='Distance between wheels in meters'
    )
    
    declare_max_linear_speed = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.5',
        description='Maximum linear speed in m/s'
    )
    
    declare_max_angular_speed = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='2.0',
        description='Maximum angular speed in rad/s'
    )
    
    declare_connection_retry_seconds = DeclareLaunchArgument(
        'connection_retry_seconds',
        default_value='5.0',
        description='How often to retry connecting to Arduino in seconds'
    )
    
    # Launch the auto Arduino bridge node
    arduino_bridge_node = Node(
        package='arduino_motor_bridge',
        executable='auto_arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'wheel_radius': wheel_radius,
            'base_width': base_width,
            'max_linear_speed': max_linear_speed,
            'max_angular_speed': max_angular_speed,
            'connection_retry_seconds': connection_retry_seconds,
        }],
        emulate_tty=True
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_serial_port,
        declare_baud_rate,
        declare_wheel_radius,
        declare_base_width,
        declare_max_linear_speed,
        declare_max_angular_speed,
        declare_connection_retry_seconds,
        arduino_bridge_node
    ])
