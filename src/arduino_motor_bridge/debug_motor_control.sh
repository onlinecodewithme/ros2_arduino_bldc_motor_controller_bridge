#!/bin/bash

# This script tests direct motor control without the navigation stack
# It sends specific velocity commands and monitors Arduino responses

set -e  # Exit on error

echo "===================================================="
echo "Arduino Motor Control Debugging Tool"
echo "===================================================="

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws_arduino/install/setup.bash

# Default configuration 
SERIAL_PORT="/dev/ttyACM1"
MODE="direct"  # "direct" or "ros2"

# Function to display usage information
usage() {
  echo "Usage: $0 [OPTIONS]"
  echo "Options:"
  echo "  --serial-port=/dev/ttyACMx  Specify Arduino port"
  echo "  --mode=direct|ros2         Test mode (direct serial or ROS2 nodes)"
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
    --mode=*)
      MODE="${1#*=}"
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

# Verify Arduino port exists
if [ ! -e "$SERIAL_PORT" ]; then
  echo "❌ ERROR: Serial port $SERIAL_PORT not found!"
  echo "Available serial ports:"
  ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No serial devices found"
  exit 1
fi

echo "Using Arduino port: $SERIAL_PORT"
echo "Test mode: $MODE"

if [ "$MODE" = "direct" ]; then
  echo "===================================================="
  echo "DIRECT SERIAL TEST MODE"
  echo "===================================================="
  echo "This will send motor commands directly through serial"
  echo "and monitor responses."
  
  # Create temporary Python script for direct serial testing
  TMP_SCRIPT="/tmp/motor_test_$$.py"
  cat > $TMP_SCRIPT << 'EOF'
#!/usr/bin/env python3

import sys
import time
import serial
import signal
import threading

def signal_handler(sig, frame):
    print("\nExiting...")
    # Stop motors before exiting
    if ser and ser.is_open:
        print("Stopping motors")
        ser.write(b"v 0 0\r\n")
        time.sleep(0.5)
        ser.close()
    sys.exit(0)

# Register signal handler
signal.signal(signal.SIGINT, signal_handler)

if len(sys.argv) < 2:
    print("Usage: python3 script.py /dev/ttyACMx")
    sys.exit(1)

port = sys.argv[1]
print(f"Opening serial port {port}")

try:
    ser = serial.Serial(
        port=port,
        baudrate=115200,
        timeout=1.0,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE
    )
except Exception as e:
    print(f"Error opening serial port: {e}")
    sys.exit(1)

print("Serial port opened successfully")

# Function to read serial in a separate thread
def serial_reader():
    while ser.is_open:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(f"ARDUINO: {line}")
            time.sleep(0.01)
        except Exception as e:
            print(f"Error reading serial: {e}")
            break

# Start reader thread
reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()

# Toggle DTR to reset Arduino
print("Resetting Arduino...")
ser.dtr = False
time.sleep(0.1)
ser.dtr = True
time.sleep(2)  # Wait for Arduino to reset and initialize

# Send initial test command
print("Sending test command...")
ser.write(b"?\r\n")
time.sleep(1)

print("\n==================================================")
print("MOTOR CONTROL TEST SEQUENCE")
print("==================================================")
print("Testing forward motion...")
ser.write(b"v 0.1 0.0\r\n")  # Low forward speed
print("CMD: v 0.1 0.0 (forward at 10% speed)")
time.sleep(3)

print("Stopping...")
ser.write(b"v 0.0 0.0\r\n")
print("CMD: v 0.0 0.0 (stop)")
time.sleep(2)

print("Testing rotation...")
ser.write(b"v 0.0 0.5\r\n")  # Medium rotation speed
print("CMD: v 0.0 0.5 (rotate at 50% speed)")
time.sleep(3)

print("Stopping...")
ser.write(b"v 0.0 0.0\r\n")
print("CMD: v 0.0 0.0 (stop)")
time.sleep(2)

print("Testing backward motion...")
ser.write(b"v -0.1 0.0\r\n")  # Low backward speed
print("CMD: v -0.1 0.0 (backward at 10% speed)")
time.sleep(3)

print("Stopping...")
ser.write(b"v 0.0 0.0\r\n")
print("CMD: v 0.0 0.0 (stop)")
time.sleep(2)

print("\n==================================================")
print("INTERACTIVE MODE")
print("==================================================")
print("Enter commands to send to the Arduino:")
print("Examples:")
print("  v 0.2 0.0   - Move forward at 20% speed")
print("  v 0.0 0.5   - Rotate at 50% speed")
print("  v -0.2 0.0  - Move backward at 20% speed")
print("  v 0.0 0.0   - Stop")
print("  ?          - Get status")
print("  q          - Quit")

while True:
    try:
        cmd = input("COMMAND> ").strip()
        if not cmd:
            continue
        if cmd.lower() == 'q':
            break
        if cmd == '?':
            ser.write(b"?\r\n")
        else:
            ser.write(f"{cmd}\r\n".encode())
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"Error: {e}")

# Stop motors and close serial
print("Stopping motors and closing serial port...")
ser.write(b"v 0.0 0.0\r\n")
time.sleep(0.5)
ser.close()
print("Done")
EOF

  # Make script executable
  chmod +x $TMP_SCRIPT

  # Run the script
  python3 $TMP_SCRIPT "$SERIAL_PORT"

  # Clean up
  rm -f $TMP_SCRIPT

else
  # ROS2 mode
  echo "===================================================="
  echo "ROS2 TEST MODE"
  echo "===================================================="
  echo "Starting Arduino bridge node and publishing test velocities"
  
  # Start the Arduino bridge node in background
  echo "Starting Arduino bridge node..."
  ros2 run arduino_motor_bridge arduino_bridge_node --ros-args -p serial_port:=$SERIAL_PORT &
  BRIDGE_PID=$!
  
  # Wait for node to initialize
  sleep 5
  
  # Create a temporary publisher script
  TMP_SCRIPT="/tmp/vel_publisher_$$.py"
  cat > $TMP_SCRIPT << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Velocity publisher initialized')
        
    def publish_velocity(self, linear_x, angular_z, duration=2.0):
        self.get_logger().info(f'Publishing velocity: linear={linear_x}, angular={angular_z} for {duration}s')
        
        # Create Twist message
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        
        # Publish for specified duration
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            self.get_logger().info(f'Sent: linear={msg.linear.x}, angular={msg.angular.z}')
            rate.sleep()
            
        # Send stop command
        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        self.get_logger().info('Sent stop command')
        time.sleep(0.5)  # Make sure stop command is received

def main():
    rclpy.init()
    node = VelocityPublisher()
    
    try:
        print("\n==================================================")
        print("MOTOR CONTROL TEST SEQUENCE")
        print("==================================================")
        
        # Forward motion test
        print("Testing forward motion...")
        node.publish_velocity(0.1, 0.0, 3.0)
        time.sleep(1.0)
        
        # Rotation test
        print("Testing rotation...")
        node.publish_velocity(0.0, 0.5, 3.0)
        time.sleep(1.0)
        
        # Backward motion test
        print("Testing backward motion...")
        node.publish_velocity(-0.1, 0.0, 3.0)
        time.sleep(1.0)
        
        print("\n==================================================")
        print("INTERACTIVE MODE")
        print("==================================================")
        print("Enter velocity commands (linear angular duration):")
        print("Examples:")
        print("  0.2 0.0 3   - Move forward at 20% speed for 3 seconds")
        print("  0.0 0.5 2   - Rotate at 50% speed for 2 seconds")
        print("  -0.2 0.0 3  - Move backward at 20% speed for 3 seconds")
        print("  q          - Quit")
        
        while True:
            cmd = input("COMMAND> ").strip()
            if not cmd:
                continue
            if cmd.lower() == 'q':
                break
                
            try:
                parts = cmd.split()
                if len(parts) >= 2:
                    linear = float(parts[0])
                    angular = float(parts[1])
                    duration = float(parts[2]) if len(parts) > 2 else 2.0
                    node.publish_velocity(linear, angular, duration)
                else:
                    print("Invalid command format. Use: linear angular [duration]")
            except ValueError:
                print("Invalid values. Use numbers for linear, angular, and duration.")
            except Exception as e:
                print(f"Error: {e}")
    
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before exit
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        time.sleep(0.5)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

  # Make script executable
  chmod +x $TMP_SCRIPT
  
  # Run the script
  python3 $TMP_SCRIPT
  
  # Clean up
  echo "Stopping Arduino bridge node..."
  kill $BRIDGE_PID 2>/dev/null || true
  rm -f $TMP_SCRIPT
fi

echo "===================================================="
echo "Debugging session completed"
echo "===================================================="
