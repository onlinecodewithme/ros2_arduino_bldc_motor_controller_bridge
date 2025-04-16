#!/bin/bash

# Start virtual display if needed 
if ! xdpyinfo -display :1 &> /dev/null; then
    echo "Starting Xvfb display server"
    Xvfb :1 -screen 0 1024x768x24 &
    XVFB_PID=$!
    trap "kill $XVFB_PID" EXIT
fi

# Set display for GUI applications
export DISPLAY=:1

# Environment setup
WORKSPACE_DIR="/home/x4/ros2_ws_arduino"
source /opt/ros/humble/setup.bash

# Copy URDF and meshes to proper locations to ensure they're found
echo "Copying URDF files to ensure they're in the right location..."
mkdir -p ${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/meshes
cp -f ${WORKSPACE_DIR}/src/robot_navigation/urdf/tracked_robot.urdf.xacro ${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/
cp -f ${WORKSPACE_DIR}/src/robot_navigation/urdf/meshes/simple_track.dae ${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/meshes/

# Build the package - skip if it fails but still try to run
cd ${WORKSPACE_DIR}
colcon build --packages-select robot_navigation || echo "Build failed, but continuing..."

# Source the workspace
source ${WORKSPACE_DIR}/install/setup.bash

# Set simulation time parameter
export USE_SIM_TIME=false

# Directly run launch file
echo "Running navigation with direct paths..."
ros2 launch robot_navigation tracked_robot_navigation.launch.py
