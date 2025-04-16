#!/bin/bash

# Script to run navigation with RViz using the auto-detecting Arduino bridge
# This script uses the original complete_navigation.launch.py file which has been updated to use auto-detection

set -e  # Exit on error

# Set display for GUI applications
export DISPLAY=:1

echo "===================================================="
echo "Navigation with Auto-Detecting Arduino Bridge"
echo "===================================================="

# Current directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$(realpath $SCRIPT_DIR/../../)"
echo "Workspace directory: $WS_DIR"

# Source ROS environment
source /opt/ros/humble/setup.bash
source $WS_DIR/install/setup.bash

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
echo "* Arduino will be automatically detected on any available port"
echo "* Visualization will be displayed using RViz"
echo ""
echo "To set a navigation goal in RViz:"
echo "1. Click on the '2D Nav Goal' button in the toolbar"
echo "2. Click and drag on the map to set position and orientation"
echo ""
echo "Press Ctrl+C to stop the navigation system"
echo "===================================================="

# Launch complete_navigation with all components
# Using the modified complete_navigation.launch.py that now uses auto_arduino_bridge
ros2 launch robot_navigation complete_navigation.launch.py \
  use_rviz:=true

echo "Navigation system stopped"
