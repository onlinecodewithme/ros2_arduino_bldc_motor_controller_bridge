#!/bin/bash

# Run navigation with Hall sensor odometry to provide robot position feedback
# This script builds the necessary packages and launches the navigation system

set -e  # Exit on error

echo "===================================================="
echo "Starting Navigation with Hall Sensor Odometry"
echo "===================================================="

# Set display for RViz and GUI tools
export DISPLAY=:1

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws_arduino/install/setup.bash

# Default configuration 
SERIAL_PORT="/dev/ttyACM1"
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

# Verify environment setup
echo "Checking environment..."
if [ "$USE_RVIZ" = "true" ] && [ -z "$DISPLAY" ]; then
  echo "Warning: No DISPLAY environment variable set, but still trying to use RViz"
else
  echo "✓ Display configured: $DISPLAY"
fi

# Check for Arduino devices if auto-detect enabled
if [ "$AUTO_PORT_DETECT" = "true" ]; then
  echo "Checking for available Arduino devices..."
  ARDUINO_DEVICES=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "none")
  
  if [ "$ARDUINO_DEVICES" = "none" ]; then
    echo "❌ Warning: No Arduino devices found. Navigation will start but motor control may not work."
  else
    echo "✓ Found Arduino devices: $ARDUINO_DEVICES"
    echo "Will auto-select first available device."
  fi
else
  echo "Using manually specified Arduino port: $SERIAL_PORT"
  if [ ! -e "$SERIAL_PORT" ]; then
    echo "❌ Warning: Specified port $SERIAL_PORT does not exist! Navigation will start but motor control may not work."
  fi
fi

# Build required packages
echo "Building packages..."
cd ~/ros2_ws_arduino
colcon build --packages-select robot_navigation arduino_motor_bridge
source ~/ros2_ws_arduino/install/setup.bash
echo "✓ Build completed"

# Launch the system
echo "===================================================="
echo "Starting navigation system with hall sensor odometry..."
echo "===================================================="

# Display configuration information
echo "Configuration:"
echo "- Arduino serial port: $SERIAL_PORT (auto-detect: $AUTO_PORT_DETECT)"
echo "- RViz visualization: $USE_RVIZ"
echo ""
echo "To set a navigation goal in RViz:"
echo "1. Click on the '2D Nav Goal' button in the toolbar"
echo "2. Click and drag on the map to set position and orientation"
echo ""
echo "Press Ctrl+C to stop the navigation system"
echo "===================================================="

# Launch the navigation system with hall sensor odometry
ros2 launch robot_navigation nav_with_hall_sensors.launch.py \
  serial_port:=$SERIAL_PORT \
  use_rviz:=$USE_RVIZ \
  auto_port_detect:=$AUTO_PORT_DETECT

echo "Navigation system stopped"
