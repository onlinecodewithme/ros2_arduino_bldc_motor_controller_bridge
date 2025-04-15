from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('arduino_motor_bridge')
    
    # Get the parameter file path
    param_file = os.path.join(pkg_dir, 'config', 'arduino_params.yaml')
    
    # Launch the Arduino bridge node
    bridge_node = Node(
        package='arduino_motor_bridge',
        executable='arduino_bridge_node',
        name='arduino_bridge',
        output='screen',
        parameters=[param_file],
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True
    )
    
    # Launch the test node after a delay to ensure the bridge is up
    test_node = Node(
        package='arduino_motor_bridge',
        executable='test_arduino_commands',
        name='arduino_test',
        output='screen',
        emulate_tty=True
    )
    
    # Add a delay before launching the test node
    delayed_test = TimerAction(
        period=5.0,  # 5 second delay to ensure bridge is ready
        actions=[test_node]
    )

    return LaunchDescription([
        bridge_node,
        delayed_test
    ])
