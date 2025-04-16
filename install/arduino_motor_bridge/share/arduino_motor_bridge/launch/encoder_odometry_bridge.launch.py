#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino connection'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0825',
        description='Wheel radius in meters'
    )
    
    base_width_arg = DeclareLaunchArgument(
        'base_width',
        default_value='0.17',
        description='Distance between wheels in meters'
    )
    
    ticks_per_rev_arg = DeclareLaunchArgument(
        'ticks_per_rev',
        default_value='4000.0',
        description='Encoder ticks per revolution (4000 for E38S with quadrature)'
    )
    
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.5',
        description='Maximum linear speed in m/s'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='2.0',
        description='Maximum angular speed in rad/s'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='20.0',
        description='Odometry publish rate in Hz'
    )

    # Create the odometry node with E38S encoders
    encoder_odometry_node = Node(
        package='arduino_motor_bridge',
        executable='encoder_odometry_node',
        name='encoder_odometry_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'base_width': LaunchConfiguration('base_width'),
            'encoder_ticks_per_rev': LaunchConfiguration('ticks_per_rev'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'cmd_vel_topic': 'cmd_vel',
            'odom_topic': 'odom'
        }],
        emulate_tty=True
    )
    
    # Return the LaunchDescription
    return LaunchDescription([
        serial_port_arg,
        wheel_radius_arg,
        base_width_arg,
        ticks_per_rev_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        publish_rate_arg,
        encoder_odometry_node
    ])
