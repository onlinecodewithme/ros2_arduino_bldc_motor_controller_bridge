#!/bin/bash

# Test script for launching navigation system with proper visualization
# This script ensures the DISPLAY is properly set and provides enhanced UI feedback

set -e  # Exit on error

# Set display for GUI applications
export DISPLAY=:1

echo "===================================================="
echo "Navigation Test with Enhanced UI Visualization"
echo "===================================================="

# Current directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$(realpath $SCRIPT_DIR/../../)"
echo "Workspace directory: $WS_DIR"

# Source ROS environment
source /opt/ros/humble/setup.bash
source $WS_DIR/install/setup.bash

# Default parameters
SERIAL_PORT="/dev/ttyACM0"

# Process arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --serial-port=*)
      SERIAL_PORT="${1#*=}"
      shift
      ;;
    --help)
      echo "Usage: $0 [--serial-port=/dev/ttyACMx]"
      echo "Run navigation system with enhanced visualization"
      exit 0
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Use --help for usage information"
      exit 1
      ;;
  esac
done

# Function to check if Arduino is connected
check_arduino() {
  if [ -e "$SERIAL_PORT" ]; then
    echo "✓ Arduino device detected at $SERIAL_PORT"
    return 0
  else
    echo "❌ Arduino device not found at $SERIAL_PORT"
    echo "Available serial devices:"
    ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No serial devices found"
    
    # Try to find alternative ports
    for port in /dev/ttyACM* /dev/ttyUSB*; do
      if [ -e "$port" ]; then
        echo "Found possible Arduino port: $port"
        echo "Try using: $0 --serial-port=$port"
      fi
    done
    
    return 1
  fi
}

# Check for required files
echo "Checking system configuration..."
CONFIG_DIR="$SCRIPT_DIR/config"
if [ ! -d "$CONFIG_DIR" ]; then
  echo "❌ Configuration directory not found!"
  exit 1
fi

if [ ! -f "$CONFIG_DIR/navigate_w_replanning.xml" ] || 
   [ ! -f "$CONFIG_DIR/navigate_through_poses.xml" ] ||
   [ ! -f "$CONFIG_DIR/nav2_visualization.rviz" ]; then
  echo "❌ Required configuration files are missing!"
  echo "Please ensure all required files are present in $CONFIG_DIR"
  exit 1
fi
echo "✓ Configuration files found"

# Check if display is available
if [ -z "$DISPLAY" ]; then
  echo "❌ DISPLAY environment variable not set!"
  echo "Setting DISPLAY=:1"
  export DISPLAY=:1
else
  echo "✓ Display configured: $DISPLAY"
fi

# Check for Arduino
check_arduino || { echo "Warning: Continuing without Arduino verification"; }

# Build the workspace
echo "Building required packages..."
cd $WS_DIR
colcon build --packages-select robot_navigation arduino_motor_bridge
source $WS_DIR/install/setup.bash
echo "✓ Build completed"

# Launch navigation system
echo "===================================================="
echo "Starting navigation system..."
echo "===================================================="
echo "* Map will be loaded from: $WS_DIR/src/robot_navigation/maps/simple_map.yaml"
echo "* Arduino will be connected on: $SERIAL_PORT"
echo "* Visualization will be displayed using RViz"
echo ""
echo "To set a navigation goal in RViz:"
echo "1. Click on the '2D Nav Goal' button in the toolbar"
echo "2. Click and drag on the map to set position and orientation"
echo ""
echo "Press Ctrl+C to stop the navigation system"
echo "===================================================="

# Launch navigation with all components
ros2 launch robot_navigation complete_navigation.launch.py \
  serial_port:=$SERIAL_PORT \
  use_rviz:=true

echo "Navigation system stopped"
