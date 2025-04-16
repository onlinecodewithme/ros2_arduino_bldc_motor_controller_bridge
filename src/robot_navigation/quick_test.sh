#!/bin/bash

# Quick test script to verify navigation and Arduino motor bridge integration
# This script starts essential navigation components and tests sending a goal

set -e

echo "===================================================="
echo "Quick Navigation Test with Arduino Bridge"
echo "===================================================="

# Set display for RViz and GUI tools
export DISPLAY=:1

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws_arduino/install/setup.bash

# Default parameters
SERIAL_PORT="/dev/ttyACM0"
GOAL_X="1.0"
GOAL_Y="1.0"
GOAL_THETA="0.0"
TEST_DURATION=20  # seconds to run the test

# Process arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --serial-port=*)
      SERIAL_PORT="${1#*=}"
      shift
      ;;
    --goal=*)
      IFS=',' read -r GOAL_X GOAL_Y GOAL_THETA <<< "${1#*=}"
      shift
      ;;
    --duration=*)
      TEST_DURATION="${1#*=}"
      shift
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Usage: $0 [--serial-port=/dev/ttyACMx] [--goal=X,Y,THETA] [--duration=SECONDS]"
      exit 1
      ;;
  esac
done

echo "Test Configuration:"
echo "- Arduino port: $SERIAL_PORT"
echo "- Navigation goal: X=$GOAL_X, Y=$GOAL_Y, θ=$GOAL_THETA"
echo "- Test duration: $TEST_DURATION seconds"
echo "===================================================="

# Function to check if Arduino is connected
check_arduino() {
  if [ -e "$SERIAL_PORT" ]; then
    echo "Arduino device detected at $SERIAL_PORT"
    return 0
  else
    echo "Error: Arduino device not found at $SERIAL_PORT"
    echo "Available serial devices:"
    ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No serial devices found"
    return 1
  fi
}

# Check for Arduino before continuing
check_arduino || exit 1

# Start in a new terminal: map server, robot state publisher, navigation stack
echo "Starting navigation stack (minimal)..."
gnome-terminal --title="Navigation Stack" -- bash -c "
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws_arduino/install/setup.bash
  ros2 launch robot_navigation navigation.launch.py use_rviz:=false
  echo 'Navigation stack stopped'
  read -p 'Press Enter to close this terminal...'
" || xterm -title "Navigation Stack" -e "
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws_arduino/install/setup.bash
  ros2 launch robot_navigation navigation.launch.py use_rviz:=false
  echo 'Navigation stack stopped'
  read -p 'Press Enter to close this terminal...'
" &

# Start in a new terminal: Arduino bridge
echo "Starting Arduino bridge..."
gnome-terminal --title="Arduino Bridge" -- bash -c "
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws_arduino/install/setup.bash
  ros2 launch arduino_motor_bridge arduino_bridge.launch.py serial_port:=$SERIAL_PORT
  echo 'Arduino bridge stopped'
  read -p 'Press Enter to close this terminal...'
" || xterm -title "Arduino Bridge" -e "
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws_arduino/install/setup.bash
  ros2 launch arduino_motor_bridge arduino_bridge.launch.py serial_port:=$SERIAL_PORT
  echo 'Arduino bridge stopped'
  read -p 'Press Enter to close this terminal...'
" &

# Give the nodes time to start
echo "Waiting for nodes to start..."
sleep 10

# Send navigation goal
echo "Sending navigation goal: X=$GOAL_X, Y=$GOAL_Y, θ=$GOAL_THETA..."
ros2 run robot_navigation test_goal_navigation $GOAL_X $GOAL_Y $GOAL_THETA &
GOAL_PID=$!

# Monitor velocity commands going to the Arduino
echo "Monitoring velocity commands for $TEST_DURATION seconds..."
gnome-terminal --title="Velocity Monitor" -- bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic echo /cmd_vel
" || xterm -title "Velocity Monitor" -e "
  source /opt/ros/humble/setup.bash
  ros2 topic echo /cmd_vel
" &
MONITOR_PID=$!

# Wait for the test duration
echo "Test running for $TEST_DURATION seconds..."
sleep $TEST_DURATION

# Clean up
echo "Test complete, cleaning up..."
kill $GOAL_PID $MONITOR_PID 2>/dev/null || true
echo "Quick test completed. Check the terminal windows for results."
echo "===================================================="
