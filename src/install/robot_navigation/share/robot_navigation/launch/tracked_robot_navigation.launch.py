import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Directory and path setup
    package_dir = get_package_share_directory('robot_navigation')
    urdf_file_path = os.path.join(package_dir, 'urdf', 'tracked_robot.urdf.xacro')
    nav2_config_path = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(package_dir, 'config', 'nav2_visualization.rviz')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(package_dir, 'maps', 'simple_map.yaml'))
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Process the XACRO file
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()
    
    # Robot State Publisher Parameters
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

    # Joint State Publisher Node - this will publish joint states for our wheels
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'rate': 30,
            'use_sim_time': False,
            'robot_description': robot_description_raw
        }],
        output='screen',
    )

    # Track Rotation Simulator - This node will simulate track rotation during navigation
    # Using Python script directly since the entry point might not be properly installed
    track_script_path = os.path.join(
        get_package_share_directory('robot_navigation'),
        '..',
        '..',
        '..',
        'src',
        'robot_navigation',
        'robot_navigation',
        'track_rotation_simulator.py'
    )
    
    track_rotation_node = Node(
        package='robot_navigation',
        executable='python3',
        name='track_rotation_simulator',
        output='screen',
        arguments=[track_script_path],
        parameters=[{
            'use_sim_time': False,
            'wheel_radius': 0.095,  # 190mm diameter wheel
            'update_rate': 30.0,
            'robot_description': robot_description_raw
        }]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': False, 'robot_description': robot_description_raw}]
    )

    # Nav2 Bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_config_path,
            'autostart': 'true'
        }.items()
    )

    # Return the full launch description
    return LaunchDescription([
        # Launch Arguments (keeping this for backward compatibility)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true - Note: Override to false internally'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(package_dir, 'maps', 'simple_map.yaml'),
            description='Full path to map yaml file'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'),
            
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        track_rotation_node,
        rviz_node,
        nav2_bringup_launch
    ])
