#!/bin/bash

# Set up for a clean environment
killall -9 rviz2 joint_state_publisher joint_state_publisher_gui robot_state_publisher 2>/dev/null

# Set display for GUI applications
export DISPLAY=:1

# Check if virtual X server is needed and available
if ! xdpyinfo -display :1 &> /dev/null; then
    echo "Starting Xvfb display server"
    Xvfb :1 -screen 0 1024x768x24 &
    XVFB_PID=$!
    trap "kill $XVFB_PID" EXIT
fi

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Workspace directory
WORKSPACE_DIR="/home/x4/ros2_ws_arduino"
cd ${WORKSPACE_DIR}

# Ensure directories exist
mkdir -p ${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/

# Copy the simplified URDF file to installation directory
echo "Copying URDF file to installation directory..."
cp -f ${WORKSPACE_DIR}/src/robot_navigation/urdf/tracked_robot_simplified.urdf ${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/

# Build the package
echo "Building robot_navigation package..."
colcon build --packages-select robot_navigation

# Source the workspace
source ${WORKSPACE_DIR}/install/setup.bash

# Create a temporary file with the robot_description parameter
URDF_CONTENT=$(cat ${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/tracked_robot_simplified.urdf)
PARAM_FILE=$(mktemp)
# Create properly formatted YAML parameter file
echo "/**:" > $PARAM_FILE
echo "  ros__parameters:" >> $PARAM_FILE
echo "    robot_description: '${URDF_CONTENT}'" >> $PARAM_FILE

echo "Starting visualization components..."

# Directly publish the robot_description parameter to a global parameter
echo "Publishing robot_description directly..."
ros2 param set /robot_description robot_description "$URDF_CONTENT" || true

# Launch robot_state_publisher in the background
ros2 run robot_state_publisher robot_state_publisher &
RSP_PID=$!
sleep 2

# Publish the robot description again just to be sure
ros2 param set /robot_description robot_description "$URDF_CONTENT" || true
sleep 1

# Launch joint_state_publisher (without params, it will pick up robot_description from the parameter server)
ros2 run joint_state_publisher joint_state_publisher &
JSP_PID=$!
sleep 1

# Launch joint_state_publisher_gui
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
GUI_PID=$!
sleep 1

# Launch RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share rviz2)/default.rviz &
RVIZ_PID=$!

echo "All components launched. You should see the robot model in RViz."
echo "You can use the joint_state_publisher_gui to move the continuous joints (wheels)."
echo "Press Ctrl+C to exit."

# Clean up on exit
trap "kill $RSP_PID $JSP_PID $GUI_PID $RVIZ_PID; rm -f $PARAM_FILE" EXIT SIGINT SIGTERM

# Wait for Ctrl+C
wait
