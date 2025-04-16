#!/bin/bash

# Test script to run just the improved Arduino bridge node
# This isolates the Arduino bridge from the navigation system for testing

set -e  # Exit on error

echo "===================================================="
echo "Testing Improved Arduino Bridge"
echo "===================================================="

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws_arduino/install/setup.bash

# Default configuration 
SERIAL_PORT="/dev/ttyACM0"
TIMEOUT=30  # How long to run the test for in seconds

# Function to display usage information
usage() {
  echo "Usage: $0 [OPTIONS]"
  echo "Options:"
  echo "  --serial-port=/dev/ttyACMx  Specify Arduino port"
  echo "  --timeout=30               How long to run the test (seconds)"
  echo "  --help                     Display this help and exit"
  exit 1
}

# Process arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --serial-port=*)
      SERIAL_PORT="${1#*=}"
      shift
      ;;
    --timeout=*)
      TIMEOUT="${1#*=}"
      shift
      ;;
    --help)
      usage
      ;;
    *)
      echo "Unknown argument: $1"
      usage
      ;;
  esac
done

# Check if Arduino port exists
if [ ! -e "$SERIAL_PORT" ]; then
  echo "❌ Serial port $SERIAL_PORT not found!"
  echo "Available serial ports:"
  ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No serial devices found"
  
  # Try to find alternative Arduino ports
  echo "Checking for alternative Arduino ports..."
  for port in /dev/ttyACM* /dev/ttyUSB*; do
    if [ -e "$port" ]; then
      echo "Found possible Arduino port: $port"
      echo "Try using: $0 --serial-port=$port"
    fi
  done
  
  echo "Continuing with default port but may not work."
fi

# Ensure the latest code is built
echo "Building arduino_motor_bridge package..."
cd ~/ros2_ws_arduino
colcon build --packages-select arduino_motor_bridge
source ~/ros2_ws_arduino/install/setup.bash

# Make sure PYTHONPATH includes our workspace
export PYTHONPATH=$PYTHONPATH:~/ros2_ws_arduino/src/arduino_motor_bridge

echo "===================================================="
echo "Starting Arduino bridge test..."
echo "===================================================="
echo "Serial port: $SERIAL_PORT"
echo "Test duration: $TIMEOUT seconds"
echo "The test will send velocity commands to the Arduino and monitor responses."
echo "Press Ctrl+C to stop the test early."
echo "===================================================="

# Create a temp script to run the test
TEST_SCRIPT="/tmp/arduino_bridge_test_$$.py"
cat > $TEST_SCRIPT << 'EOF'
#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
import threading
import sys
import signal
import os

# Get parameters from environment
serial_port = os.environ.get('TEST_SERIAL_PORT', '/dev/ttyACM0')
timeout = float(os.environ.get('TEST_TIMEOUT', '30'))

class ArduinoBridgeTest(Node):
    def __init__(self):
        super().__init__('arduino_bridge_test')
        
        # Set up QoS profile for cmd_vel
        cmd_vel_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publisher for cmd_vel
        self.vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            cmd_vel_qos
        )
        
        # Create a timer for velocity commands
        self.vel_timer = self.create_timer(1.0, self.publish_velocity)
        
        # Pattern for velocity commands: forward, stop, backward, turn left, turn right, stop
        self.cmd_patterns = [
            (0.1, 0.0),   # Forward
            (0.0, 0.0),   # Stop
            (-0.1, 0.0),  # Backward
            (0.0, 0.5),   # Turn left
            (0.0, -0.5),  # Turn right
            (0.0, 0.0),   # Stop
        ]
        self.pattern_index = 0
        
        # Start time
        self.start_time = time.time()
        
        self.get_logger().info(f"Testing Arduino bridge with serial port: {serial_port}")
        self.get_logger().info(f"Test will run for {timeout} seconds")
        self.get_logger().info("Starting velocity command publishing...")
        
    def publish_velocity(self):
        """Publish velocity commands in a pattern."""
        # Get current pattern
        linear_x, angular_z = self.cmd_patterns[self.pattern_index]
        
        # Create Twist message
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        # Publish message
        self.vel_pub.publish(msg)
        
        # Log what we're doing
        cmd_type = "STOP"
        if linear_x > 0:
            cmd_type = "FORWARD"
        elif linear_x < 0:
            cmd_type = "BACKWARD"
        elif angular_z > 0:
            cmd_type = "LEFT TURN"
        elif angular_z < 0:
            cmd_type = "RIGHT TURN"
            
        self.get_logger().info(f"Published velocity: {cmd_type} (linear={linear_x:.2f}, angular={angular_z:.2f})")
        
        # Move to next pattern
        self.pattern_index = (self.pattern_index + 1) % len(self.cmd_patterns)
        
        # Check if test is complete
        elapsed = time.time() - self.start_time
        if elapsed >= timeout:
            self.get_logger().info(f"Test completed after {elapsed:.1f} seconds")
            # Send stop command
            stop_msg = Twist()
            self.vel_pub.publish(stop_msg)
            self.get_logger().info("Sent final STOP command")
            sys.exit(0)


def main():
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\nTest interrupted by user.")
        if rclpy.ok():
            # Send stop command before exiting
            node = rclpy.create_node('temp_stop_node')
            stop_pub = node.create_publisher(Twist, 'cmd_vel', 10)
            stop_msg = Twist()
            stop_pub.publish(stop_msg)
            node.get_logger().info("Sent emergency STOP command")
            time.sleep(0.5)  # Give time for the message to be processed
            node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize ROS
    rclpy.init()
    node = ArduinoBridgeTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        stop_msg = Twist()
        node.vel_pub.publish(stop_msg)
        node.get_logger().info("Sent final STOP command")
        
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

# Make the test script executable
chmod +x $TEST_SCRIPT

# Start the improved Arduino bridge node
{
# Export variables for the test script
export TEST_SERIAL_PORT=$SERIAL_PORT
export TEST_TIMEOUT=$TIMEOUT

# Run the bridge in background
echo "Starting improved Arduino bridge..."
python3 -m arduino_motor_bridge.improved_bridge_node --ros-args -p serial_port:=$SERIAL_PORT &
BRIDGE_PID=$!

# Wait a bit for the bridge to start
sleep 3

# Run the test script
echo "Starting test client..."
python3 $TEST_SCRIPT

# Clean up
kill $BRIDGE_PID 2>/dev/null || true
rm -f $TEST_SCRIPT
} || {
# Clean up on error
rm -f $TEST_SCRIPT
echo "Test failed!"
exit 1
}

echo "Test completed successfully!"
