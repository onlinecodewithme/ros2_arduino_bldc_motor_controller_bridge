#!/bin/bash
# Test navigation with simulated odometry
# This script runs the full navigation stack with simulated odometry in place of real encoders

set -e

echo "===================================================="
echo "Starting Navigation with Simulated Odometry"
echo "===================================================="

# Set display for RViz
export DISPLAY=:1

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws_arduino/install/setup.bash

# Build packages with symlink install to ensure latest changes
echo "Building packages..."
cd ~/ros2_ws_arduino
colcon build --packages-select robot_navigation --symlink-install
source ~/ros2_ws_arduino/install/setup.bash

# Kill any existing processes
pkill -f "nav2" || true
pkill -f "rviz" || true
pkill -f "robot_state_publisher" || true
sleep 1

echo "===================================================="
echo "Starting simulated odometry publisher..."
# Start simulated odometry publisher in background
ros2 run robot_navigation simulated_odometry &
ODOM_PID=$!
sleep 2

echo "Starting navigation visualization..."
# Use the robot_state_publisher with URDF file
URDF_PATH=$(ros2 pkg prefix robot_navigation)/share/robot_navigation/urdf/diff_robot.urdf.xacro
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $URDF_PATH)" &
RSP_PID=$!

# Start static TF publisher
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
TF_PID=$!

# Start map server
MAP_PATH=$(ros2 pkg prefix robot_navigation)/share/robot_navigation/maps/simple_map.yaml
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP_PATH &
MAP_PID=$!

sleep 3
echo "Starting RViz..."
# Start RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz &
RVIZ_PID=$!

echo "===================================================="
echo "System is running!"
echo "Set a navigation goal in RViz using the 2D Nav Goal button"
echo "Press Ctrl+C to stop all components"
echo "===================================================="

# Wait for Ctrl+C
trap "echo 'Shutting down...'; kill $ODOM_PID $RSP_PID $TF_PID $MAP_PID $RVIZ_PID; exit 0" INT TERM
wait
