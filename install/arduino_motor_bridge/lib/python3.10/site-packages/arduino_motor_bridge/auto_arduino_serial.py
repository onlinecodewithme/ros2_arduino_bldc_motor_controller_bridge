#!/usr/bin/env python3

import os
import glob
import time
import threading
import subprocess
from typing import List, Optional, Callable

from arduino_motor_bridge.improved_arduino_serial import ImprovedArduinoSerial

class AutoArduinoSerial(ImprovedArduinoSerial):
    """
    Extension of ImprovedArduinoSerial that automatically finds available Arduino ports
    and handles permission issues.
    """
    
    def __init__(self, port='auto', baudrate=115200, timeout=0.1, logger=None):
        """
        Initialize with automatic port detection.
        
        Args:
            port: 'auto' for automatic detection, or specific port name
            baudrate: Communication speed
            timeout: Read timeout in seconds
            logger: ROS logger instance
        """
        # Initialize with empty port, we'll set it during connect
        super().__init__(port='', baudrate=baudrate, timeout=timeout, logger=logger)
        
        # Store the original port parameter to check if auto detection is requested
        self.port_param = port
        self.last_scan_time = 0
        self.scan_ports_interval = 5.0  # seconds between port scans
        self.arduino_ports = []
        self.current_port_index = 0
        self.port_permission_fixed = False
    
    def connect(self, retry_count=3, retry_delay=2.0):
        """
        Find Arduino ports and establish connection with auto-retry.
        
        Args:
            retry_count: Number of connection attempts before moving to next port
            retry_delay: Delay between retries in seconds
            
        Returns:
            bool: True if connection was successful
        """
        # For a specific port, just use the parent class behavior
        if self.port_param != 'auto':
            self.port = self.port_param
            self.logger.info(f"Using specified port: {self.port}")
            return super().connect(retry_count, retry_delay)
        
        # Auto-detect and try all available Arduino ports
        if len(self.arduino_ports) == 0 or time.time() - self.last_scan_time > self.scan_ports_interval:
            self._scan_for_arduino_ports()
        
        if not self.arduino_ports:
            self.logger.error("No Arduino ports found during auto-detection")
            return False
        
        # Try each port in sequence with the specified retry count
        for i in range(len(self.arduino_ports)):
            port_index = (self.current_port_index + i) % len(self.arduino_ports)
            port = self.arduino_ports[port_index]
            
            if not self._check_port_permissions(port):
                self.logger.warn(f"Insufficient permissions for {port}. Attempting to fix...")
                if self._fix_port_permissions(port):
                    self.logger.info(f"Fixed permissions for {port}")
                else:
                    self.logger.error(f"Failed to fix permissions for {port}")
                    continue
            
            self.port = port
            self.logger.info(f"Attempting connection on auto-detected port: {self.port}")
            
            if super().connect(retry_count, retry_delay):
                self.current_port_index = port_index  # Remember the successful port index
                return True
        
        self.logger.error("Failed to connect to any available Arduino ports")
        return False
    
    def reconnect(self, retry_count=3, retry_delay=2.0):
        """
        Attempt to reconnect, possibly on a different port if auto-detection is enabled.
        
        Args:
            retry_count: Number of connection attempts before giving up
            retry_delay: Delay between retries in seconds
            
        Returns:
            bool: True if reconnection was successful
        """
        # If we're not using auto-detection, just use the standard connect
        if self.port_param != 'auto':
            return super().connect(retry_count, retry_delay)
        
        # Move to the next port index for the next attempt
        self.current_port_index = (self.current_port_index + 1) % max(1, len(self.arduino_ports))
        
        # Re-scan for ports if needed
        if time.time() - self.last_scan_time > self.scan_ports_interval:
            self._scan_for_arduino_ports()
        
        return self.connect(retry_count, retry_delay)
    
    def _scan_for_arduino_ports(self):
        """
        Scan for available Arduino ports.
        Updates the arduino_ports list with all potential Arduino devices.
        """
        self.logger.info("Scanning for Arduino ports...")
        
        # Common patterns for Arduino devices
        patterns = [
            '/dev/ttyACM*',  # Most Arduino boards
            '/dev/ttyUSB*',  # Arduino with FTDI or similar USB-serial adapters
            '/dev/ttyAMA*',  # Serial port on Raspberry Pi GPIO
            '/dev/ttyS*'     # Standard serial ports that might be used
        ]
        
        # Find all matching ports
        ports = []
        for pattern in patterns:
            ports.extend(glob.glob(pattern))
        
        # Sort the ports to get consistent order
        ports.sort()
        
        self.arduino_ports = ports
        self.last_scan_time = time.time()
        
        if ports:
            self.logger.info(f"Found {len(ports)} potential Arduino ports: {', '.join(ports)}")
        else:
            self.logger.warn("No Arduino ports found. Is the device connected?")
    
    def _check_port_permissions(self, port):
        """
        Check if we have permission to access the specified port.
        
        Args:
            port: Port to check
            
        Returns:
            bool: True if we have read/write access
        """
        try:
            return os.access(port, os.R_OK | os.W_OK)
        except Exception as e:
            self.logger.error(f"Error checking permissions for {port}: {e}")
            return False
    
    def _fix_port_permissions(self, port):
        """
        Attempt to fix permissions for the specified port.
        
        This may require sudo access and dialout group membership.
        
        Args:
            port: Port to fix
            
        Returns:
            bool: True if permissions were fixed successfully
        """
        if self.port_permission_fixed:
            # We already tried to fix permissions in this session, don't retry
            return False
        
        try:
            # Check if user is in dialout group
            username = os.environ.get('USER', os.environ.get('USERNAME', 'unknown'))
            self.logger.info(f"Checking if user '{username}' is in dialout group...")
            
            # First try a safer approach by making the port accessible to all users
            # This requires sudo access, so it may fail
            self.logger.info(f"Attempting to fix permissions for {port}...")
            
            # Try to change the permissions of the port
            cmd = f"sudo chmod a+rw {port}"
            try:
                subprocess.run(cmd, shell=True, check=True, timeout=5)
                self.logger.info(f"Changed permissions for {port}")
                self.port_permission_fixed = True
                return True
            except subprocess.SubprocessError as e:
                self.logger.warn(f"Failed to change permissions with command: {cmd}. Error: {e}")
            
            # Provide advice for permanent fix
            self.logger.warn("For a permanent fix, add your user to the dialout group:")
            self.logger.warn("    sudo usermod -a -G dialout $USER")
            self.logger.warn("Then log out and log back in.")
            
            return False
        except Exception as e:
            self.logger.error(f"Error fixing permissions: {e}")
            return False
    
    def _handle_error(self):
        """
        Override error handling to consider reconnecting to a different port.
        """
        # Use the parent error handling first
        super()._handle_error()
        
        # If we're using auto-detection and get consecutive errors,
        # consider switching to a different port next time
        if self.port_param == 'auto' and self.error_count > 3:
            self.logger.info("Multiple errors detected, will try different port on next connection attempt")
            self.current_port_index = (self.current_port_index + 1) % max(1, len(self.arduino_ports))
    
    def _reconnect_thread(self):
        """
        Override reconnection thread to use our enhanced reconnect method.
        """
        try:
            self.logger.info("Starting reconnection attempt...")
            
            # Wait a bit before attempting reconnection
            time.sleep(2.0)
            
            # Close any existing connection
            self._close_existing_connection()
            
            # Try to connect with our enhanced method
            if self.reconnect(retry_count=5, retry_delay=1.0):
                self.logger.info("Reconnected successfully to Arduino")
            else:
                self.logger.error("Failed to reconnect to Arduino")
        except Exception as e:
            self.logger.error(f"Error in reconnection thread: {e}")
        finally:
            self.is_reconnecting = False


# Simple test if run directly
if __name__ == '__main__':
    import argparse
    import rclpy
    from rclpy.node import Node
    
    parser = argparse.ArgumentParser(description='Test auto Arduino serial communication')
    parser.add_argument('--port', default='auto', help='Serial port (default: auto)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--fix-permissions', action='store_true', help='Try to fix port permissions')
    args = parser.parse_args()
    
    # Initialize rclpy for logging
    rclpy.init()
    node = Node('auto_arduino_serial_test')
    logger = node.get_logger()
    
    # Create serial interface
    arduino = AutoArduinoSerial(port=args.port, baudrate=args.baud, logger=logger)
    
    # Connect to Arduino
    if arduino.connect():
        try:
            # Set up callbacks
            arduino.set_odometry_callback(lambda l, r, ld, rd: 
                logger.info(f"Odometry: left={l}, right={r}, left_dist={ld:.3f}, right_dist={rd:.3f}"))
            
            arduino.set_debug_callback(lambda msg: 
                logger.info(f"Debug: {msg}"))
            
            # Send a test command
            arduino.send_manual_command('?')  # Request menu
            
            # Keep node running
            rclpy.spin(node)
        except KeyboardInterrupt:
            logger.info("Test interrupted")
        finally:
            arduino.disconnect()
            node.destroy_node()
            rclpy.shutdown()
    else:
        logger.error("Failed to connect to Arduino")
        node.destroy_node()
        rclpy.shutdown()
