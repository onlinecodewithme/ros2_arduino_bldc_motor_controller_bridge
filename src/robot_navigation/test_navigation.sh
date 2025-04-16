#!/bin/bash

# Test script for navigation components
# This script focuses on testing the basic components needed for navigation

echo "===================================================="
echo "Starting Robot Navigation Test"
echo "===================================================="

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws_arduino/install/setup.bash

# Test robot state publisher only
echo "Testing Robot State Publisher..."
ros2 launch robot_navigation robot_state_publisher.launch.py
