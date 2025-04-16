#!/bin/bash

# This script restarts the navigation system with the improved configuration
# and sends a test goal to verify the motor control

set -e  # Exit on error

echo "===================================================="
echo "Restarting Navigation and Testing Goal Navigation"
echo "===================================================="

# 1. Stop any running navigation processes
echo "Stopping any running navigation processes..."
pkill -f "navigation" || true  # Don't exit if no processes found
pkill -f "ros2_ws_arduino" || true
sleep 2

# 2. Rebuild the packages
echo "Rebuilding packages..."
cd ~/ros2_ws_arduino
colcon build --packages-select robot_navigation arduino_motor_bridge
source ~/ros2_ws_arduino/install/setup.bash
echo "✓ Build completed"

# 3. Launch the improved navigation in background
echo "Starting improved navigation system in background..."
./src/robot_navigation/run_improved_navigation.sh --no-rviz &
NAV_PID=$!

# Wait to make sure navigation has time to start
echo "Waiting for navigation system to start (15 seconds)..."
sleep 15

# 4. Send a test goal
echo "===================================================="
echo "Sending test navigation goal..."
echo "===================================================="

# Set display for GUI tools if needed
export DISPLAY=:1

# Open a new terminal for the test goal
echo "Running test goal navigation with coordinates (1.0, 1.0)..."
gnome-terminal -- bash -c "cd ~/ros2_ws_arduino && source install/setup.bash && python3 src/robot_navigation/test_goal_navigation.py 1.0 1.0; read -p 'Press Enter to continue...'"

echo ""
echo "Navigation test initiated."
echo "You can observe velocity commands in the terminal."
echo ""
echo "NOTE: Arduino bridge will convert velocity commands to motor control signals"
echo "Press CTRL+C to stop all processes when done testing."
echo "===================================================="

# Wait for user to stop the test
wait $NAV_PID

echo "Navigation system stopped"
