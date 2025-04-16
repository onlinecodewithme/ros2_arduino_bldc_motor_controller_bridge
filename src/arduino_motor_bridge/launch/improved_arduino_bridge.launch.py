#!/usr/bin/env python3

"""
Launch file for the improved Arduino bridge node with enhanced reconnection and error handling.
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
    
    # Declare launch arguments
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino connection'
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
    
    # Launch the improved Arduino bridge node using the original arduino_bridge_node
    # but with the improved parameters - this avoids the need for a new executable entry
    arduino_bridge_node = Node(
        package='arduino_motor_bridge',
        executable='arduino_bridge_node',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'wheel_radius': wheel_radius,
            'base_width': base_width,
            'max_linear_speed': max_linear_speed,
            'max_angular_speed': max_angular_speed,
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
        arduino_bridge_node
    ])
