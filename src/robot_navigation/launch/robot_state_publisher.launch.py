#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch the robot state publisher node which publishes robot transforms.
    This is necessary for the navigation stack to know the robot's geometry.
    """
    
    # Get the package directory
    pkg_dir = get_package_share_directory('robot_navigation')
    
    # Path to the URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'diff_robot.urdf.xacro')
    
    # Use xacro to process the file
    robot_description_command = Command([
        'xacro ', urdf_file
    ])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch arguments declaration
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock if true'
    )

    # Create robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_command
        }]
    )

    # Create a static transform publisher
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Return launch description
    return LaunchDescription([
        declare_use_sim_time_argument,
        robot_state_publisher_node,
        static_transform_publisher
    ])
