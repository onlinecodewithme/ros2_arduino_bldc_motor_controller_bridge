#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch the robot state publisher node which publishes robot transforms.
    This is necessary for the navigation stack to know the robot's geometry.
    """
    
    # Robot description - a simple differential drive robot
    robot_description = """
    <?xml version="1.0"?>
    <robot name="arduino_robot">
        <link name="base_footprint"/>
        
        <link name="base_link">
            <visual>
                <geometry>
                    <box size="0.4 0.3 0.1"/>
                </geometry>
                <material name="blue">
                    <color rgba="0 0 0.8 1"/>
                </material>
            </visual>
            <collision>
                <geometry>
                    <box size="0.4 0.3 0.1"/>
                </geometry>
            </collision>
        </link>
        
        <link name="lidar_link">
            <visual>
                <geometry>
                    <cylinder length="0.05" radius="0.05"/>
                </geometry>
                <material name="black">
                    <color rgba="0 0 0 1"/>
                </material>
            </visual>
        </link>
        
        <joint name="base_joint" type="fixed">
            <parent link="base_footprint"/>
            <child link="base_link"/>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
        </joint>
        
        <joint name="lidar_joint" type="fixed">
            <parent link="base_link"/>
            <child link="lidar_link"/>
            <origin xyz="0.15 0 0.075" rpy="0 0 0"/>
        </joint>
    </robot>
    """
    
    # Generate a temporary URDF file
    urdf_file_path = '/tmp/arduino_robot.urdf'
    with open(urdf_file_path, 'w') as urdf_file:
        urdf_file.write(robot_description)
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock if true')
    
    # Create robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # Create a simple odometry node
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
        static_transform_publisher,
    ])
