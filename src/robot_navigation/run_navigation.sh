#!/bin/bash

# Run the complete navigation system with Arduino motor bridge
# This script starts all components: robot state publisher, navigation stack, 
# Arduino bridge, and RViz visualization

set -e  # Exit on error

echo "===================================================="
echo "Starting Complete Robot Navigation System"
echo "===================================================="

# Set display for RViz and GUI tools
export DISPLAY=:1

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws_arduino/install/setup.bash

# Default configuration 
SERIAL_PORT="/dev/ttyACM0"
USE_RVIZ="true"
AUTO_PORT_DETECT="true"

# Function to display usage information
usage() {
  echo "Usage: $0 [OPTIONS]"
  echo "Options:"
  echo "  --serial-port=/dev/ttyACMx  Manually specify Arduino serial port"
  echo "  --no-rviz                   Disable RViz visualization"
  echo "  --no-auto-port              Disable automatic Arduino port detection"
  echo "  --help                      Display this help and exit"
  exit 1
}

# Process arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --serial-port=*)
      SERIAL_PORT="${1#*=}"
      AUTO_PORT_DETECT="false"  # If manual port specified, disable auto detection
      shift
      ;;
    --no-rviz)
      USE_RVIZ="false"
      shift
      ;;
    --no-auto-port)
      AUTO_PORT_DETECT="false"
      shift
      ;;
    --help)
      usage
      ;;
    *)
      echo "Error: Unknown argument: $1"
      usage
      ;;
  esac
done

# Ensure required directories exist
BT_CONFIG_DIR="$HOME/ros2_ws_arduino/src/robot_navigation/config"
if [ ! -d "$BT_CONFIG_DIR" ]; then
  echo "Error: Behavior tree configuration directory not found at: $BT_CONFIG_DIR"
  exit 1
fi

# Check for required behavior tree files
if [ ! -f "$BT_CONFIG_DIR/navigate_w_replanning.xml" ] || 
   [ ! -f "$BT_CONFIG_DIR/navigate_through_poses.xml" ]; then
  echo "Error: Required behavior tree XML files not found!"
  exit 1
fi

# Check for display (required for RViz)
if [ "$USE_RVIZ" = "true" ] && [ -z "$DISPLAY" ]; then
  echo "Warning: No DISPLAY environment variable set, disabling RViz"
  USE_RVIZ="false"
fi

echo "Configuration:"
echo "- Arduino serial port: $SERIAL_PORT"
echo "- Auto-detect port: $AUTO_PORT_DETECT"
echo "- RViz visualization: $USE_RVIZ"
echo "===================================================="

# Build the workspace
echo "Building workspace..."
cd ~/ros2_ws_arduino
colcon build --packages-select robot_navigation arduino_motor_bridge
source ~/ros2_ws_arduino/install/setup.bash

# Start the navigation system
echo "Starting navigation system..."
ros2 launch robot_navigation complete_navigation.launch.py \
  serial_port:=$SERIAL_PORT \
  use_rviz:=$USE_RVIZ \
  auto_port_detect:=$AUTO_PORT_DETECT

echo "Navigation system stopped"
