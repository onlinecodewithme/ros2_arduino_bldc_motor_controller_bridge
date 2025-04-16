#!/usr/bin/env python3

"""
Hall Sensor Odometry Node

This node reads hall sensor data from the Arduino motor controller,
converts it to odometry information, and publishes it to the ROS2 navigation stack.
It provides the missing feedback loop for the navigation system.

Usage:
  ros2 run arduino_motor_bridge hall_sensor_odometry --ros-args -p serial_port:=/dev/ttyACM1
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import serial
import time
import threading
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat

class HallSensorOdometry(Node):
    """
    Processes hall sensor data from Arduino and publishes odometry.
    
    This node:
    1. Connects to Arduino
    2. Reads hall sensor pulses
    3. Converts to wheel movement
    4. Calculates robot position change
    5. Publishes odometry data
    """
    
    def __init__(self):
        super().__init__('hall_sensor_odometry')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.065)  # meters
        self.declare_parameter('base_width', 0.17)     # meters
        self.declare_parameter('ticks_per_rev', 12)    # Hall sensor pulses per wheel revolution
        self.declare_parameter('publish_rate', 20.0)   # Hz
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.base_width = self.get_parameter('base_width').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Set up QoS profiles
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            'wheel_odom',  # Separate topic from /odom for comparison/debugging
            odom_qos
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Serial connection
        self.serial = None
        self.lock = threading.Lock()
        self.running = False
        self.connected = False
        
        # State variables
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Connect to Arduino
        self.connect_to_arduino()
        
        # Create timer for publishing odometry
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odometry)
        
        # Create timer for requesting hall sensor data
        self.request_timer = self.create_timer(0.05, self.request_encoder_data)
        
        self.get_logger().info('Hall Sensor Odometry node initialized')
    
    def connect_to_arduino(self):
        """Establish connection to the Arduino."""
        try:
            self.get_logger().info(f"Connecting to Arduino on {self.serial_port}")
            
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.5,
                write_timeout=0.5
            )
            
            # Reset Arduino by toggling DTR
            self.get_logger().info("Resetting Arduino...")
            self.serial.dtr = False
            time.sleep(0.1)
            self.serial.dtr = True
            time.sleep(2.0)  # Wait for Arduino to reset and initialize
            
            # Flush buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # Send command to enable encoder reporting
            self.serial.write(b"e 1\r\n")
            self.get_logger().info("Enabled encoder reporting")
            
            # Start reading thread
            self.running = True
            self.connected = True
            self.read_thread = threading.Thread(target=self.read_serial)
            self.read_thread.daemon = True
            self.read_thread.start()
            
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.connected = False
            return False
    
    def read_serial(self):
        """Read data from Arduino in a separate thread."""
        self.get_logger().info("Serial read thread started")
        
        while self.running and self.connected:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.process_line(line)
                else:
                    time.sleep(0.001)  # Avoid busy wait
            except serial.SerialException as e:
                self.get_logger().error(f"Serial exception: {e}")
                self.connected = False
                break
            except Exception as e:
                self.get_logger().error(f"Error reading serial: {e}")
                time.sleep(0.1)  # Avoid flooding logs
        
        self.get_logger().info("Serial read thread stopped")
    
    def process_line(self, line):
        """Process a line from the Arduino."""
        try:
            # Look for encoder data in format "E,left_ticks,right_ticks"
            if line.startswith('E,'):
                parts = line.split(',')
                if len(parts) >= 3:
                    with self.lock:
                        # Update ticks
                        self.left_ticks = int(parts[1])
                        self.right_ticks = int(parts[2])
                        
                        # Log occasionally for debugging
                        if self.left_ticks % 100 == 0 or self.right_ticks % 100 == 0:
                            self.get_logger().info(f"Hall sensors: L={self.left_ticks}, R={self.right_ticks}")
            # Also process debug info
            elif line.startswith('D,') or line.startswith('DEBUG'):
                self.get_logger().info(f"Arduino Debug: {line}")
            else:
                self.get_logger().debug(f"Arduino: {line}")
        except Exception as e:
            self.get_logger().error(f"Error processing line: {e}")
    
    def request_encoder_data(self):
        """Request encoder data from Arduino."""
        if not self.connected:
            return
        
        try:
            with self.lock:
                # Send command to get encoder data
                if self.serial and self.serial.is_open:
                    self.serial.write(b"e\r\n")
        except Exception as e:
            self.get_logger().error(f"Error requesting encoder data: {e}")
    
    def publish_odometry(self):
        """Calculate and publish odometry data based on hall sensor readings."""
        if not self.connected:
            return
        
        with self.lock:
            # Get current ticks
            left_ticks = self.left_ticks
            right_ticks = self.right_ticks
            
            # Calculate delta ticks
            delta_left = left_ticks - self.last_left_ticks
            delta_right = right_ticks - self.last_right_ticks
            
            # Update last ticks
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
        
        # Calculate wheel rotations
        left_wheel_rotation = (2.0 * math.pi * delta_left) / self.ticks_per_rev
        right_wheel_rotation = (2.0 * math.pi * delta_right) / self.ticks_per_rev
        
        # Calculate wheel distances
        left_distance = left_wheel_rotation * self.wheel_radius
        right_distance = right_wheel_rotation * self.wheel_radius
        
        # Calculate robot movement
        if abs(left_distance - right_distance) < 1e-6:
            # Robot moved straight
            distance = (left_distance + right_distance) / 2.0
            delta_x = distance * math.cos(self.theta)
            delta_y = distance * math.sin(self.theta)
            delta_theta = 0.0
        else:
            # Robot moved in an arc
            distance = (left_distance + right_distance) / 2.0
            delta_theta = (right_distance - left_distance) / self.base_width
            
            # Calculate change in position
            if abs(delta_theta) < 1e-6:
                # Avoid division by zero
                delta_x = distance * math.cos(self.theta)
                delta_y = distance * math.sin(self.theta)
            else:
                # Calculate arc
                radius = distance / delta_theta
                
                # Calculate change in position
                delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
                delta_y = radius * (-math.cos(self.theta + delta_theta) + math.cos(self.theta))
        
        # Update robot position
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to -pi to pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Get current time
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'  # Standard frame name
        odom.child_frame_id = 'base_link'  # Standard frame name
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation using quaternion
        q = euler2quat(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Set velocities
        if dt > 0.0001:  # Avoid division by zero
            linear_vel = distance / dt
            angular_vel = delta_theta / dt
            
            odom.twist.twist.linear.x = linear_vel
            odom.twist.twist.angular.z = angular_vel
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        
        # Broadcast transform
        self.broadcast_transform(odom)
    
    def broadcast_transform(self, odom_msg):
        """Broadcast transform from odom to base_link."""
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id
        
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def shutdown(self):
        """Shutdown the node and close connections."""
        self.get_logger().info("Shutting down Hall Sensor Odometry node")
        
        # Stop threads
        self.running = False
        
        # Wait for read thread
        if hasattr(self, 'read_thread') and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        
        # Close serial connection
        if self.serial and self.serial.is_open:
            # Disable encoder reporting
            self.serial.write(b"e 0\r\n")
            time.sleep(0.1)
            self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    node = HallSensorOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
