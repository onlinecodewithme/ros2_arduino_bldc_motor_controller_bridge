#!/bin/bash
# Script to run complex map navigation with E38S encoder integration
# This script provides accurate robot positioning based on high-resolution encoders

set -e  # Exit on error

# Default values
MAP_PATH="$(pwd)/src/robot_navigation/maps/complex_map.yaml"
SERIAL_PORT="/dev/ttyACM0"
WHEEL_RADIUS=0.0825
BASE_WIDTH=0.17
ENCODER_TICKS=4000.0
MAX_LINEAR_SPEED=0.5
MAX_ANGULAR_SPEED=2.0
PUBLISH_RATE=20.0
USE_RVIZ=true
DISPLAY_ENV=":1"

# Process arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --no-rviz)
      USE_RVIZ=false
      shift
      ;;
    --map=*)
      MAP_PATH="${1#*=}"
      shift
      ;;
    --port=*)
      SERIAL_PORT="${1#*=}"
      shift
      ;;
    --wheel-radius=*)
      WHEEL_RADIUS="${1#*=}"
      shift
      ;;
    --base-width=*)
      BASE_WIDTH="${1#*=}"
      shift
      ;;
    --ticks-per-rev=*)
      ENCODER_TICKS="${1#*=}"
      shift
      ;;
    --max-linear-speed=*)
      MAX_LINEAR_SPEED="${1#*=}"
      shift
      ;;
    --max-angular-speed=*)
      MAX_ANGULAR_SPEED="${1#*=}"
      shift
      ;;
    --publish-rate=*)
      PUBLISH_RATE="${1#*=}"
      shift
      ;;
    --display=*)
      DISPLAY_ENV="${1#*=}"
      shift
      ;;
    *)
      echo "Unknown parameter: $1"
      echo "Usage: $0 [--no-rviz] [--map=PATH] [--port=PORT] [--wheel-radius=RADIUS] [--base-width=WIDTH] [--ticks-per-rev=TICKS] [--max-linear-speed=SPEED] [--max-angular-speed=SPEED] [--publish-rate=RATE] [--display=:X]"
      exit 1
      ;;
  esac
done

# Check if a process is using the serial port and kill it if necessary
if [ -e "$SERIAL_PORT" ]; then
  SERIAL_PIDS=$(fuser "$SERIAL_PORT" 2>/dev/null || true)
  if [ -n "$SERIAL_PIDS" ]; then
    echo "Found processes using $SERIAL_PORT: $SERIAL_PIDS"
    echo "Attempting to terminate these processes..."
    fuser -k -TERM "$SERIAL_PORT" 2>/dev/null || true
    sleep 2
  fi
fi

# Set display for GUI applications
export DISPLAY=$DISPLAY_ENV
echo "Set DISPLAY to $DISPLAY_ENV"

# Current directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$(realpath $SCRIPT_DIR/../../)"
echo "Workspace directory: $WS_DIR"

# Source ROS environment
source /opt/ros/humble/setup.bash
source $WS_DIR/install/setup.bash

# Set up trap to call cleanup on script exit
cleanup() {
  echo "Cleaning up..."
  # First try to kill with TERM
  pkill -TERM -f ros2 || true
  echo "Cleanup complete"
}
trap cleanup EXIT

echo "===================================================="
echo "Complex Map Navigation with E38S Encoder Integration"
echo "===================================================="
echo "Using map: $MAP_PATH"
echo "Using robust Arduino connection with E38S encoders"
echo "Serial port: $SERIAL_PORT"
echo "Encoder resolution: $ENCODER_TICKS ticks/revolution"
echo "Wheel radius: $WHEEL_RADIUS m, Base width: $BASE_WIDTH m"

# Launch navigation with robust encoder node
if [ "$USE_RVIZ" = true ]; then
  # Important: Launch only robust_encoder_node (not the bridge) to avoid transform conflicts
  echo "Launching complex map navigation with RViz visualization..."
  
  # Launch the robust encoder node first to establish the odom -> base_link transform
  ros2 run arduino_motor_bridge robust_encoder_node \
    --ros-args -p serial_port:=$SERIAL_PORT \
    -p wheel_radius:=$WHEEL_RADIUS \
    -p base_width:=$BASE_WIDTH \
    -p encoder_ticks_per_rev:=$ENCODER_TICKS \
    -p max_linear_speed:=$MAX_LINEAR_SPEED \
    -p max_angular_speed:=$MAX_ANGULAR_SPEED \
    -p publish_rate:=$PUBLISH_RATE &
  
  # Wait for the encoder node to establish transforms
  echo "Waiting for encoder node to initialize..."
  sleep 3
  
  # Then launch the navigation and visualization
  ros2 launch robot_navigation complex_map_navigation.launch.py \
    use_rviz:=true \
    map:=$MAP_PATH &
  
  echo "===================================================="
  echo "Navigation system started"
  echo ""
  echo "To see the robot position update:"
  echo "1. Manually rotate the encoder wheels"
  echo "2. The robot model in RViz should rotate accordingly"
  echo ""
  echo "To set a navigation goal in RViz:"
  echo "1. Click on the '2D Nav Goal' button in the toolbar"
  echo "2. Click and drag on the map to set position and orientation"
  echo ""
  echo "Press Ctrl+C to stop the navigation system"
  echo "===================================================="
else
  # Launch without RViz
  echo "Launching complex map navigation (no visualization)..."
  
  # Launch the robust encoder node first to establish the odom -> base_link transform
  ros2 run arduino_motor_bridge robust_encoder_node \
    --ros-args -p serial_port:=$SERIAL_PORT \
    -p wheel_radius:=$WHEEL_RADIUS \
    -p base_width:=$BASE_WIDTH \
    -p encoder_ticks_per_rev:=$ENCODER_TICKS \
    -p max_linear_speed:=$MAX_LINEAR_SPEED \
    -p max_angular_speed:=$MAX_ANGULAR_SPEED \
    -p publish_rate:=$PUBLISH_RATE &
  
  # Wait for the encoder node to establish transforms
  echo "Waiting for encoder node to initialize..."
  sleep 3
  
  # Then launch just the navigation stack
  ros2 launch robot_navigation complex_map_navigation.launch.py \
    use_rviz:=false \
    map:=$MAP_PATH &
  
  echo "Navigation system started (no visualization)"
  echo "Press Ctrl+C to stop"
fi

# Wait for the user to press Ctrl+C
wait
