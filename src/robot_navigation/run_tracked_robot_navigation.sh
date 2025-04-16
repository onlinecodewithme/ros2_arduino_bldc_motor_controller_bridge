#!/bin/bash

# Use Xvfb for GUI applications if needed
if ! xdpyinfo -display :1 &> /dev/null; then
    echo "Starting Xvfb display server"
    Xvfb :1 -screen 0 1024x768x24 &
    XVFB_PID=$!
    trap "kill $XVFB_PID" EXIT
fi

# Set display for GUI applications
export DISPLAY=:1

# Source ROS2 environment first
source /opt/ros/humble/setup.bash

# Build the packages
cd $(dirname $0)/..
colcon build --packages-select robot_navigation

# Source the workspace with absolute path
WORKSPACE_DIR="/home/x4/ros2_ws_arduino"
source ${WORKSPACE_DIR}/install/setup.bash

# Fix for joint_state_publisher error about invalid robot_description
export USE_SIM_TIME=false

# Launch the tracked robot navigation
ros2 launch robot_navigation tracked_robot_navigation.launch.py
