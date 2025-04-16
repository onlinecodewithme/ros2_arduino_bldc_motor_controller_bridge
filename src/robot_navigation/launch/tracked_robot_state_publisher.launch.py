import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('robot_navigation'),
        'urdf',
        'tracked_robot.urdf.xacro'
    )
    
    # Process the XACRO file
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()
    
    # Parameters for the robot_state_publisher
    robot_state_publisher_params = {
        'robot_description': robot_description_raw,
        'publish_frequency': 30.0,
        'use_sim_time': False
    }
    
    # Make robot description available as a parameter
    robot_description = {'robot_description': robot_description_raw}
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_state_publisher_params]
    )

    # Joint State Publisher Node - This will publish joint states for our wheels
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'rate': 30, 'use_sim_time': False, 'robot_description': robot_description_raw}],
        output='screen',
    )

    # If you want to visualize random joint movements in a static case
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': robot_description_raw}],
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    # Launch Argument for enabling the GUI
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui'
    )

    # RVIZ Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': robot_description_raw}],
        arguments=['-d', os.path.join(get_package_share_directory('robot_navigation'), 'config', 'nav2_visualization.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Flag to enable rviz2'
    )

    # Return the launch description
    return LaunchDescription([
        use_gui_arg,
        use_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
