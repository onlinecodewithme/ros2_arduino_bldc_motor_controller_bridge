#!/bin/bash

# Diagnostic script to test Arduino bridge connection
# This isolates the Arduino bridge node to diagnose connection issues

set -e  # Exit on error

export DISPLAY=:1  # Set display for any GUI elements

echo "===================================================="
echo "Arduino Bridge Connection Diagnostic"
echo "===================================================="

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws_arduino/install/setup.bash

# Default port
SERIAL_PORT="/dev/ttyACM0"
VERBOSE=false
RESET_ARDUINO=false

# Process arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --port=*)
      SERIAL_PORT="${1#*=}"
      shift
      ;;
    --verbose)
      VERBOSE=true
      shift
      ;;
    --reset-arduino)
      RESET_ARDUINO=true
      shift
      ;;
    --help)
      echo "Usage: $0 [OPTIONS]"
      echo "Options:"
      echo "  --port=/dev/ttyXXX   Specify Arduino port (default: /dev/ttyACM0)"
      echo "  --verbose            Show verbose debug output"
      echo "  --reset-arduino      Reset the Arduino before connecting"
      echo "  --help               Show this help message"
      exit 0
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Use --help for usage information"
      exit 1
      ;;
  esac
done

# Check if Arduino port exists
if [ ! -e "$SERIAL_PORT" ]; then
  echo "❌ Error: Arduino device not found at $SERIAL_PORT"
  echo "Available serial devices:"
  ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No serial devices found"
  
  # Try to find alternative Arduino ports
  echo "Checking for alternative Arduino ports..."
  for port in /dev/ttyACM* /dev/ttyUSB*; do
    if [ -e "$port" ]; then
      echo "Found possible Arduino port: $port"
      echo "Try using: $0 --port=$port"
    fi
  done
  
  exit 1
fi

echo "✓ Found Arduino device at $SERIAL_PORT"

# Show Arduino port permissions
echo "Arduino port permissions:"
ls -l $SERIAL_PORT

# Check if current user has appropriate permissions
if [[ ! -r "$SERIAL_PORT" || ! -w "$SERIAL_PORT" ]]; then
  echo "❌ Warning: You may not have read/write permissions for $SERIAL_PORT"
  echo "Consider adding your user to the 'dialout' group:"
  echo "  sudo usermod -a -G dialout $USER"
  echo "Then log out and log back in for changes to take effect."
fi

# Check if port is in use by another process
PORT_USERS=$(lsof $SERIAL_PORT 2>/dev/null)
if [ -n "$PORT_USERS" ]; then
  echo "❌ Warning: Serial port $SERIAL_PORT is already in use by another process:"
  echo "$PORT_USERS"
  echo "This may cause connection issues."
  
  read -p "Do you want to kill these processes? (y/n) " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    lsof $SERIAL_PORT | awk 'NR>1 {print $2}' | xargs kill -9 2>/dev/null || true
    echo "Processes terminated."
    sleep 1
  fi
fi

# Reset Arduino if requested
if [ "$RESET_ARDUINO" = true ]; then
  echo "Resetting Arduino..."
  # Use stty to toggle DTR line which resets Arduino
  stty -F $SERIAL_PORT hupcl
  sleep 1
  stty -F $SERIAL_PORT -hupcl
  sleep 2
  echo "Arduino reset complete."
fi

# Build the arduino_motor_bridge package
echo "Building arduino_motor_bridge package..."
cd ~/ros2_ws_arduino
colcon build --packages-select arduino_motor_bridge
source ~/ros2_ws_arduino/install/setup.bash

# Set up command based on verbosity level
CMD="ros2 run arduino_motor_bridge arduino_bridge_node --ros-args -p serial_port:=$SERIAL_PORT"
if [ "$VERBOSE" = true ]; then
  CMD="$CMD --ros-args --log-level debug"
fi

echo "Starting Arduino bridge node..."
echo "Command: $CMD"
echo "Press Ctrl+C to stop the node"
echo "===================================================="

# Run the Arduino bridge node
eval $CMD

echo "Arduino bridge diagnostic complete"
