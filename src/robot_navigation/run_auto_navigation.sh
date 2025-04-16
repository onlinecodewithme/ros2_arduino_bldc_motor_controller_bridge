#!/bin/bash

# Script to build and run the full navigation stack with auto-detecting Arduino bridge

# Stop script on errors
set -e

# Get directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "==== Building necessary packages ===="
cd "$WORKSPACE_DIR"
colcon build --packages-select arduino_motor_bridge robot_navigation

echo "==== Sourcing setup files ===="
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

echo "==== Running auto-detecting navigation stack ===="
echo "This navigation stack will automatically find and connect to the Arduino"
echo "on any available port and handle permission issues."
echo ""
echo "Press Ctrl+C to stop."
echo ""

# Run the auto_navigation launch file
ros2 launch robot_navigation auto_navigation.launch.py
