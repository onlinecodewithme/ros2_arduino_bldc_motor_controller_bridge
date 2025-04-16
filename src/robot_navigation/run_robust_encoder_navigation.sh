#!/bin/bash
# Script to run navigation with robust encoder implementation
# This script handles serial port recovery and improved encoder integration

# Stop script on error
set -e

# Default values
MAP_PATH="$(pwd)/src/robot_navigation/maps/simple_map.yaml"
SERIAL_PORT="/dev/ttyACM0"
WHEEL_RADIUS=0.0825
BASE_WIDTH=0.17
ENCODER_TICKS=4000.0
MAX_LINEAR_SPEED=0.5
MAX_ANGULAR_SPEED=2.0
PUBLISH_RATE=20.0
USE_RVIZ=true
DISPLAY_ENV=""

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

echo "Using map: $MAP_PATH"
echo "Using robust Arduino connection with E38S encoders"
echo "Serial port: $SERIAL_PORT"
echo "Encoder resolution: $ENCODER_TICKS ticks/revolution"

# Ensure DISPLAY is set for RViz
if [ -n "$DISPLAY_ENV" ]; then
  export DISPLAY=$DISPLAY_ENV
  echo "Set DISPLAY to $DISPLAY_ENV"
elif [ -z "$DISPLAY" ]; then
  # If DISPLAY is not set and no custom value provided, use a default
  export DISPLAY=:1
  echo "DISPLAY not set. Using default DISPLAY=:1"
fi

# More targeted approach to free the serial port
if [ -e "$SERIAL_PORT" ]; then
  echo "Checking for processes using serial port..."
  SERIAL_PIDS=$(fuser "$SERIAL_PORT" 2>/dev/null || true)
  if [ -n "$SERIAL_PIDS" ]; then
    echo "Found processes using $SERIAL_PORT: $SERIAL_PIDS"
    echo "Attempting to gracefully terminate these processes..."
    # First try graceful termination
    fuser -k -TERM "$SERIAL_PORT" 2>/dev/null || true
    sleep 2
  else
    echo "No processes found using $SERIAL_PORT"
  fi
fi

# Source ROS environment
source /opt/ros/humble/setup.bash
source $(pwd)/install/setup.bash

# Variables to track running processes
RVIZ_PID=""
ENCODER_PID=""

# Function to clean up on exit
cleanup() {
  echo "Cleaning up..."
  # Kill specific processes by PID instead of broad pkill
  if [ -n "$RVIZ_PID" ]; then
    echo "Terminating RViz process..."
    kill -TERM $RVIZ_PID 2>/dev/null || true
  fi
  if [ -n "$ENCODER_PID" ]; then
    echo "Terminating encoder process..."
    kill -TERM $ENCODER_PID 2>/dev/null || true
  fi
  echo "Cleanup complete"
}

# Set up trap to call cleanup on script exit
trap cleanup EXIT

# Launch navigation with robust encoder node
if [ "$USE_RVIZ" = true ]; then
  # Launch with visualization
  echo "Launching navigation with visualization..."
  ros2 launch robot_navigation navigation_rviz.launch.py \
    map:=$MAP_PATH \
    encoder_odometry:=true \
    serial_port:=$SERIAL_PORT \
    wheel_radius:=$WHEEL_RADIUS \
    base_width:=$BASE_WIDTH \
    ticks_per_rev:=$ENCODER_TICKS \
    max_linear_speed:=$MAX_LINEAR_SPEED \
    max_angular_speed:=$MAX_ANGULAR_SPEED \
    publish_rate:=$PUBLISH_RATE &
  
  RVIZ_PID=$!
  echo "RViz navigation launched with PID: $RVIZ_PID"
  
  # Allow more time for visualization to start
  echo "Waiting for RViz to initialize..."
  sleep 5
fi

# Launch robust encoder node
echo "Launching robust encoder node..."
ros2 launch arduino_motor_bridge robust_encoder_bridge.launch.py \
  serial_port:=$SERIAL_PORT \
  wheel_radius:=$WHEEL_RADIUS \
  base_width:=$BASE_WIDTH \
  encoder_ticks_per_rev:=$ENCODER_TICKS \
  max_linear_speed:=$MAX_LINEAR_SPEED \
  max_angular_speed:=$MAX_ANGULAR_SPEED \
  publish_rate:=$PUBLISH_RATE &

ENCODER_PID=$!
echo "Encoder node launched with PID: $ENCODER_PID"

echo "All components started. Press Ctrl+C to stop."

# Wait for the user to press Ctrl+C
wait
