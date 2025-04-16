#!/bin/bash

# Script to run navigation with the complex map using the auto-detecting Arduino bridge
# This script ensures the DISPLAY is properly set and provides enhanced UI feedback

set -e  # Exit on error

# Set display for GUI applications
export DISPLAY=:1

echo "===================================================="
echo "Navigation with Complex Map and Auto-Detecting Arduino Bridge"
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
MAP_FILE="$SCRIPT_DIR/maps/complex_map.yaml"

if [ ! -d "$CONFIG_DIR" ]; then
  echo "❌ Configuration directory not found!"
  exit 1
fi

if [ ! -f "$MAP_FILE" ]; then
  echo "❌ Complex map file not found!"
  echo "Please run 'python3 src/robot_navigation/create_complex_map.py' first"
  exit 1
fi

echo "✓ Configuration files found"
echo "✓ Complex map file found"

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
colcon build --packages-select robot_navigation
source $WS_DIR/install/setup.bash
echo "✓ Build completed"

# Launch navigation system
echo "===================================================="
echo "Starting navigation system with complex map..."
echo "===================================================="
echo "* Complex map will be loaded from: $MAP_FILE"
echo "* Map size: 20.0 x 20.0 meters with complex obstacles"
echo "* Arduino will be automatically detected on any available port"
echo "* Visualization will be displayed using RViz"
echo ""
echo "To set a navigation goal in RViz:"
echo "1. Click on the '2D Nav Goal' button in the toolbar"
echo "2. Click and drag on the map to set position and orientation"
echo ""
echo "Press Ctrl+C to stop the navigation system"
echo "===================================================="

# Launch navigation with complex map
ros2 launch robot_navigation complex_map_navigation.launch.py \
  use_rviz:=true \
  map:="$MAP_FILE"

echo "Navigation system stopped"
