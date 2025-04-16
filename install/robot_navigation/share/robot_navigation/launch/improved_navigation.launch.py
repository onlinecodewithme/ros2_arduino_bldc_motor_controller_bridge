#!/usr/bin/env python3

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the complete navigation system with improved Arduino motor bridge.
    
    This launch file integrates:
    1. The robot state publisher for TF frames
    2. The Nav2 navigation stack with map server
    3. The improved Arduino motor bridge with better reliability
    4. RViz for visualization
    """
    # Get package directories
    nav_pkg_dir = get_package_share_directory('robot_navigation')
    arduino_pkg_dir = get_package_share_directory('arduino_motor_bridge')
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_port_detect = LaunchConfiguration('auto_port_detect')
    
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')
    
    declare_serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino connection')
    
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz if true')
        
    declare_auto_port_detect_arg = DeclareLaunchArgument(
        'auto_port_detect',
        default_value='true',
        description='Automatically detect Arduino port if true')
    
    # If auto_port_detect is true, use a script to find the Arduino port
    arduino_port_finder = ExecuteProcess(
        cmd=['bash', '-c', 'for p in /dev/ttyACM* /dev/ttyUSB*; do [ -e "$p" ] && echo "Found Arduino port: $p" && echo "$p" && exit 0; done; echo "No Arduino ports found, using default"; echo "/dev/ttyACM0"'],
        name='arduino_port_finder',
        output='screen',
        condition=IfCondition(auto_port_detect)
    )
    
    # Include robot state publisher launch
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg_dir, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include improved Arduino bridge launch
    arduino_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(arduino_pkg_dir, 'launch', 'improved_arduino_bridge.launch.py')
        ]),
        launch_arguments={
            'serial_port': serial_port
        }.items()
    )
    
    # Include navigation launch (with map server, AMCL, planners, etc.)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg_dir, 'launch', 'navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Create path to custom rviz config
    rviz_config_dir = os.path.join(nav_pkg_dir, 'config')
    rviz_config_file = os.path.join(rviz_config_dir, 'nav2_visualization.rviz')
    
    # Ensure navigation-specific behavior tree files exist
    bt_xml_dir = os.path.join(nav_pkg_dir, 'config')
    os.makedirs(bt_xml_dir, exist_ok=True)
    
    # Check and create the behavior tree files if they don't exist
    navigate_w_replanning_xml = os.path.join(bt_xml_dir, 'navigate_w_replanning.xml')
    if not os.path.exists(navigate_w_replanning_xml):
        with open(navigate_w_replanning_xml, 'w') as f:
            f.write('''
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ReactiveFallback name="ComputePathToPoseRecoveryFallback">
              <GoalUpdated/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            </ReactiveFallback>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ReactiveFallback name="FollowPathRecoveryFallback">
            <GoalUpdated/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
          </ReactiveFallback>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.3" backup_speed="0.05"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
''')
    
    navigate_through_poses_xml = os.path.join(bt_xml_dir, 'navigate_through_poses.xml')
    if not os.path.exists(navigate_through_poses_xml):
        with open(navigate_through_poses_xml, 'w') as f:
            f.write('''
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathThroughPoses">
            <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"/>
            <ReactiveFallback name="ComputePathThroughPosesRecoveryFallback">
              <GoalUpdated/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            </ReactiveFallback>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ReactiveFallback name="FollowPathRecoveryFallback">
            <GoalUpdated/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
          </ReactiveFallback>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.3" backup_speed="0.05"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
''')
    
    # Direct RViz launch with improved visualization
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Navigation CLI for command-line control of goals
    navigation_cli_node = Node(
        package='robot_navigation',
        executable='navigation_cli',
        name='navigation_cli',
        output='screen',
        emulate_tty=True,
        condition=IfCondition('false')  # Disabled by default
    )
    
    # Return the launch description
    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_serial_port_arg,
        declare_use_rviz_arg,
        declare_auto_port_detect_arg,
        arduino_port_finder,
        robot_state_publisher_launch,
        arduino_bridge_launch,
        navigation_launch,
        rviz_node,
        navigation_cli_node
    ])
