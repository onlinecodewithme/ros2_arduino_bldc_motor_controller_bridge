#!/bin/bash

# Start the ROS2 Navigation System with Arduino motor bridge integration
# This script starts all the components needed for autonomous navigation

echo "===================================================="
echo "Starting Robot Navigation System"
echo "===================================================="
echo "Activating navigation components sequentially"
echo "===================================================="

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws_arduino/install/setup.bash

# Function to check if a node is running
check_node() {
  ros2 node list | grep -q "$1"
  return $?
}

# Set display for RViz
export DISPLAY=:1

echo "Step 1: Starting Robot State Publisher..."
ros2 launch robot_navigation robot_state_publisher.launch.py &
ROBOT_STATE_PID=$!
sleep 3
if ! check_node "robot_state_publisher"; then
  echo "ERROR: Robot state publisher failed to start"
else
  echo "Robot state publisher started successfully"
fi

echo "Step 2: Starting Map Server..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$(ros2 pkg prefix robot_navigation)/share/robot_navigation/maps/simple_map.yaml &
MAP_SERVER_PID=$!
sleep 2

echo "Step 3: Starting Map Server Lifecycle Manager..."
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:=['map_server'] -p autostart:=true &
MAP_LIFECYCLE_PID=$!
sleep 2

echo "Step 4: Starting Navigation2 Core Components..."
ros2 launch nav2_bringup navigation_launch.py params_file:=$(ros2 pkg prefix robot_navigation)/share/robot_navigation/config/nav2_params.yaml &
NAV2_PID=$!
sleep 5

echo "Step 5: Activating Navigation2 Components..."
# Wait a bit longer for components to fully initialize
sleep 5

echo "Step 6: Starting the Navigation CLI..."
echo "===================================================="
echo "Navigation CLI Interface"
echo "Available commands:"
echo "  goto X Y [THETA] - Navigate to position X,Y with optional orientation"
echo "  cancel          - Cancel the current navigation goal"
echo "  help            - Show more commands"
echo "  exit            - Exit the program"
echo "===================================================="
ros2 run robot_navigation navigation_cli

# When navigation CLI exits, kill the other components
echo "Shutting down navigation stack..."
kill $NAV2_PID
kill $MAP_LIFECYCLE_PID
kill $MAP_SERVER_PID
kill $ROBOT_STATE_PID
echo "Navigation system stopped"
