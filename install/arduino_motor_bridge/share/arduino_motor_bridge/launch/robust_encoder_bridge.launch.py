import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('arduino_motor_bridge')
    
    # Parameters for the encoder node
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.0825')
    base_width = LaunchConfiguration('base_width', default='0.17')
    encoder_ticks_per_rev = LaunchConfiguration('encoder_ticks_per_rev', default='4000.0')
    
    # Create launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino communication'
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
    
    encoder_ticks_arg = DeclareLaunchArgument(
        'encoder_ticks_per_rev',
        default_value='4000.0',
        description='Encoder ticks per revolution'
    )
    
    # Optional - Release the serial port if it's locked
    release_port = ExecuteProcess(
        cmd=['bash', '-c', 'lsof -t /dev/ttyACM0 | xargs -r kill -9'],
        output='screen',
        shell=True
    )
    
    # Robust encoder bridge node
    robust_encoder_node = Node(
        package='arduino_motor_bridge',
        executable='robust_encoder_node',
        name='robust_encoder_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': 115200,
            'wheel_radius': wheel_radius,
            'base_width': base_width,
            'encoder_ticks_per_rev': encoder_ticks_per_rev,
            'timeout': 1.0,
            'retry_count': 5,
            'watchdog_timer': 2.0,
            'publish_tf': True,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'left_wheel_frame': 'left_wheel',
            'right_wheel_frame': 'right_wheel'
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        wheel_radius_arg,
        base_width_arg,
        encoder_ticks_arg,
        release_port,
        robust_encoder_node
    ])
