#!/usr/bin/env python3

"""
Test script for the Hall Sensor Odometry node.
This script directly runs the hall_sensor_odometry module.
"""

import sys
import os
import time
import rclpy

# Add the package directory to Python path to find the module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from arduino_motor_bridge.hall_sensor_odometry import main

if __name__ == '__main__':
    print("Starting Hall Sensor Odometry Test...")
    print(f"Using serial port: {sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM1'}")
    
    # Modify sys.argv for the main function if needed
    if len(sys.argv) <= 1:
        sys.argv.append("--ros-args")
        sys.argv.append("-p")
        sys.argv.append("serial_port:=/dev/ttyACM1")
    
    try:
        # Run the main function from the hall_sensor_odometry module
        main()
    except KeyboardInterrupt:
        print("Test interrupted by user")
        sys.exit(0)
