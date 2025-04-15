"""Launch file for arduino_motor_bridge node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for arduino_motor_bridge node."""
    # Get the launch directory
    pkg_dir = get_package_share_directory('arduino_motor_bridge')
    config_dir = os.path.join(pkg_dir, 'config')
    
    # Create launch configuration variables
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    wheel_separation = LaunchConfiguration('wheel_separation')
    wheel_radius = LaunchConfiguration('wheel_radius')
    use_tf = LaunchConfiguration('use_tf')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    publish_rate = LaunchConfiguration('publish_rate')
    
    # Declare the launch arguments
    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino')
    
    declare_baud_rate_cmd = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication')
    
    declare_wheel_separation_cmd = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.35',
        description='Distance between wheels in meters')
    
    declare_wheel_radius_cmd = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0825',
        description='Wheel radius in meters')
    
    declare_use_tf_cmd = DeclareLaunchArgument(
        'use_tf',
        default_value='true',
        description='Whether to publish TF transform')
    
    declare_odom_frame_cmd = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID')
    
    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame ID')
    
    declare_publish_rate_cmd = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate at which to publish odometry')
    
    # Create the arduino_bridge node
    arduino_bridge_node = Node(
        package='arduino_motor_bridge',
        executable='arduino_bridge_node',  # No .py extension
        name='arduino_bridge',
        prefix='python3 -u',  # Ensure proper execution with Python interpreter
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'wheel_separation': wheel_separation,
            'wheel_radius': wheel_radius,
            'use_tf': use_tf,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'publish_rate': publish_rate,
        }]
    )
    
    # Create the launch description and add declared launch arguments and nodes
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_serial_port_cmd)
    ld.add_action(declare_baud_rate_cmd)
    ld.add_action(declare_wheel_separation_cmd)
    ld.add_action(declare_wheel_radius_cmd)
    ld.add_action(declare_use_tf_cmd)
    ld.add_action(declare_odom_frame_cmd)
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_publish_rate_cmd)
    
    # Add the node to the launch description
    ld.add_action(arduino_bridge_node)
    
    return ld
