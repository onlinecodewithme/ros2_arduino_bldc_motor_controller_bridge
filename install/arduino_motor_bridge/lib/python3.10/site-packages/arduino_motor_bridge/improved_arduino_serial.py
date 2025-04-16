#!/usr/bin/env python3

import struct
import time
import threading
import serial
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import os

class ImprovedArduinoSerial:
    """
    Enhanced serial communication with Arduino motor controller that provides
    better error handling and reconnection capabilities.
    """
    
    # Protocol constants (must match Arduino code)
    COMMAND_HEADER = b'#'
    VELOCITY_HEADER = b'V'
    ODOMETRY_HEADER = b'O'
    RESET_HEADER = b'R'
    ACK_HEADER = b'A'
    DEBUG_HEADER = b'D'
    
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=0.1, logger=None):
        """
        Initialize serial connection to Arduino with improved error handling.
        
        Args:
            port: Serial port name
            baudrate: Communication speed
            timeout: Read timeout in seconds
            logger: ROS logger instance
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.connected = False
        self.last_error_time = 0
        self.error_count = 0
        self.logger = logger or get_logger('arduino_serial')
        self.read_thread = None
        self.running = False
        self.reconnect_thread = None
        self.is_reconnecting = False
        self.odometry_callback = None
        self.debug_callback = None
        self.lock = threading.Lock()
        
    def connect(self, retry_count=3, retry_delay=2.0):
        """
        Establish serial connection to Arduino with multiple retries.
        
        Args:
            retry_count: Number of connection attempts before giving up
            retry_delay: Delay between retries in seconds
            
        Returns:
            bool: True if connection was successful
        """
        for attempt in range(retry_count):
            try:
                self.logger.info(f"Connection attempt {attempt+1}/{retry_count} to Arduino on {self.port}")
                
                # First try to close if already open
                self._close_existing_connection()
                
                # Wait before retry
                if attempt > 0:
                    time.sleep(retry_delay)
                
                # Create new serial connection with exclusive access
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    write_timeout=1.0,
                    exclusive=True  # Lock the port to prevent other processes from using it
                )
                
                # Reset Arduino by toggling DTR line
                self._reset_arduino()
                
                # Send a test command and verify response
                if self._test_connection():
                    # Start the read thread
                    self.connected = True
                    self.error_count = 0
                    self.running = True
                    self.read_thread = threading.Thread(target=self._read_loop)
                    self.read_thread.daemon = True
                    self.read_thread.start()
                    self.logger.info(f"Successfully connected to Arduino on {self.port}")
                    return True
                
                self.logger.warning(f"Arduino test response failed on attempt {attempt+1}")
                self._close_existing_connection()
                
            except serial.SerialException as e:
                self.logger.error(f"Serial exception on attempt {attempt+1}: {e}")
                self._close_existing_connection()
                
            except Exception as e:
                self.logger.error(f"Unexpected error on attempt {attempt+1}: {e}")
                self._close_existing_connection()
        
        self.logger.error(f"Failed to connect to Arduino after {retry_count} attempts")
        self.connected = False
        return False
    
    def _close_existing_connection(self):
        """
        Safely close any existing serial connection.
        """
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.logger.info("Closed existing serial connection")
        except Exception as e:
            self.logger.warning(f"Error closing existing connection: {e}")
        
        # Clear serial object
        self.serial = None
        self.connected = False
    
    def _reset_arduino(self):
        """
        Reset Arduino by toggling DTR line with proper timing.
        """
        self.logger.info("Resetting Arduino with DTR toggle...")
        try:
            # First make sure DTR is high
            self.serial.dtr = True
            time.sleep(0.1)
            
            # Toggle DTR low to reset
            self.serial.dtr = False
            time.sleep(0.5)
            
            # Back to high
            self.serial.dtr = True
            
            # Wait for Arduino bootloader and setup
            time.sleep(2.5)
            
            # Flush buffers
            self.logger.info("Flushing serial buffers...")
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.1)
        except Exception as e:
            self.logger.warning(f"Error during Arduino reset: {e}")
    
    def _test_connection(self):
        """
        Test serial connection by sending a test command and checking response.
        
        Returns:
            bool: True if test was successful
        """
        try:
            self.logger.info("Testing Arduino connection...")
            
            # Send test command
            self.serial.write(self.COMMAND_HEADER + b'?')
            time.sleep(0.5)
            
            # Read response
            response = self.serial.read(self.serial.in_waiting or 1)
            self.logger.info(f"Arduino test response length: {len(response)}")
            
            # Consider test successful if Arduino responds with any data
            return len(response) > 0
        except Exception as e:
            self.logger.error(f"Connection test failed: {e}")
            return False
    
    def disconnect(self):
        """
        Close serial connection and clean up threads.
        """
        self.logger.info("Disconnecting from Arduino...")
        
        # Stop threads
        self.running = False
        
        # Wait for read thread to finish
        if self.read_thread and self.read_thread.is_alive():
            self.logger.info("Waiting for read thread to stop...")
            self.read_thread.join(timeout=2.0)
        
        # Wait for reconnect thread to finish
        if self.reconnect_thread and self.reconnect_thread.is_alive():
            self.logger.info("Waiting for reconnect thread to stop...")
            self.reconnect_thread.join(timeout=2.0)
        
        # Close serial connection
        self._close_existing_connection()
        self.logger.info("Disconnected from Arduino")
    
    def send_velocity_command(self, left_vel, right_vel):
        """
        Send velocity command to Arduino with error handling.
        
        Args:
            left_vel: Left wheel velocity in m/s
            right_vel: Right wheel velocity in m/s
        
        Returns:
            bool: True if command sent successfully
        """
        if not self.connected:
            self.logger.warning("Not connected to Arduino - skipping velocity command")
            return False
        
        try:
            with self.lock:
                # Pack two floats into binary format
                left_bytes = struct.pack('<f', left_vel)
                right_bytes = struct.pack('<f', right_vel)
                
                # Send command: VELOCITY_HEADER + left_bytes + right_bytes
                self.serial.write(self.VELOCITY_HEADER + left_bytes + right_bytes)
                self.logger.debug(f"Sent velocity command: L={left_vel:.2f}, R={right_vel:.2f}")
                return True
        except Exception as e:
            self.logger.error(f"Failed to send velocity command: {e}")
            self._handle_error()
            return False
    
    def send_reset_command(self):
        """
        Send reset command to reset odometry counters.
        
        Returns:
            bool: True if command sent successfully
        """
        if not self.connected:
            self.logger.warning("Not connected to Arduino - skipping reset command")
            return False
        
        try:
            with self.lock:
                self.serial.write(self.RESET_HEADER)
                self.logger.debug("Sent reset command")
                return True
        except Exception as e:
            self.logger.error(f"Failed to send reset command: {e}")
            self._handle_error()
            return False
    
    def send_debug_request(self):
        """
        Request debug information from Arduino.
        
        Returns:
            bool: True if request sent successfully
        """
        if not self.connected:
            self.logger.warning("Not connected to Arduino - skipping debug request")
            return False
        
        try:
            with self.lock:
                self.serial.write(self.DEBUG_HEADER)
                self.logger.debug("Sent debug request")
                return True
        except Exception as e:
            self.logger.error(f"Failed to send debug request: {e}")
            self._handle_error()
            return False
    
    def send_manual_command(self, command_char):
        """
        Send a manual command for testing.
        
        Args:
            command_char: Single character command (w,a,s,d,q,e,etc.)
        
        Returns:
            bool: True if command sent successfully
        """
        if not self.connected:
            self.logger.warning("Not connected to Arduino - skipping manual command")
            return False
        
        try:
            with self.lock:
                self.serial.write(self.COMMAND_HEADER + command_char.encode('ascii'))
                self.logger.debug(f"Sent manual command: {command_char}")
                return True
        except Exception as e:
            self.logger.error(f"Failed to send manual command: {e}")
            self._handle_error()
            return False
    
    def set_odometry_callback(self, callback):
        """
        Set callback for odometry data.
        
        Args:
            callback: Function to call with odometry data
        """
        self.odometry_callback = callback
    
    def set_debug_callback(self, callback):
        """
        Set callback for debug information.
        
        Args:
            callback: Function to call with debug information
        """
        self.debug_callback = callback
    
    def _handle_error(self):
        """
        Handle connection errors with potential reconnection.
        """
        now = time.time()
        
        # Increment error count
        self.error_count += 1
        
        # If we're getting frequent errors, mark connection as lost
        if now - self.last_error_time < 1.0 and self.error_count > 3:
            self.logger.error("Multiple errors detected, connection appears to be lost")
            self.connected = False
            
            # Try to reconnect in a separate thread to avoid blocking
            if not self.is_reconnecting:
                self.is_reconnecting = True
                self.reconnect_thread = threading.Thread(target=self._reconnect_thread)
                self.reconnect_thread.daemon = True
                self.reconnect_thread.start()
        
        self.last_error_time = now
    
    def _reconnect_thread(self):
        """
        Background thread for handling reconnection.
        """
        try:
            self.logger.info("Starting reconnection attempt...")
            
            # Wait a bit before attempting reconnection
            time.sleep(2.0)
            
            # Close any existing connection
            self._close_existing_connection()
            
            # Try to connect with retries
            if self.connect(retry_count=5, retry_delay=1.0):
                self.logger.info("Reconnected successfully to Arduino")
            else:
                self.logger.error("Failed to reconnect to Arduino")
        except Exception as e:
            self.logger.error(f"Error in reconnection thread: {e}")
        finally:
            self.is_reconnecting = False
    
    def _read_loop(self):
        """
        Background thread for reading serial data with improved error handling.
        """
        self.logger.info("Serial read thread started")
        consecutive_errors = 0
        
        while self.running:
            try:
                if not self.connected:
                    time.sleep(0.5)  # Don't spin CPU if disconnected
                    continue
                
                if self.serial.in_waiting:
                    # Read a line with timeout
                    line = self.serial.readline().decode('ascii', errors='ignore').strip()
                    
                    if not line:
                        continue
                    
                    # Reset error counter on successful read
                    consecutive_errors = 0
                    
                    # Process received data based on header
                    if line.startswith(self.ODOMETRY_HEADER.decode()):
                        self._process_odometry(line)
                    elif line.startswith(self.DEBUG_HEADER.decode()):
                        self._process_debug(line)
                    elif line.startswith(self.ACK_HEADER.decode()):
                        self.logger.debug(f"Received ACK: {line}")
                    else:
                        self.logger.debug(f"Received: {line}")
                else:
                    # Sleep briefly to avoid CPU hogging
                    time.sleep(0.005)
            except serial.SerialException as e:
                self.logger.error(f"Serial exception in read thread: {e}")
                consecutive_errors += 1
                self._handle_error()
                time.sleep(0.1)
            except Exception as e:
                self.logger.error(f"Error in serial read thread: {e}")
                consecutive_errors += 1
                
                # If we get too many consecutive errors, try to reconnect
                if consecutive_errors > 10:
                    self.connected = False
                    self.logger.error("Too many consecutive errors, marking connection as lost")
                    self._handle_error()
                
                # Sleep longer to avoid rapid error logging
                time.sleep(0.1)
        
        self.logger.info("Serial read thread stopped")
    
    def _process_odometry(self, line):
        """
        Process odometry data from Arduino.
        
        Args:
            line: Odometry data line from Arduino
        """
        try:
            # Format: O,left_pos,right_pos,left_dist,right_dist
            parts = line.split(',')
            if len(parts) >= 5:
                # Parse data
                left_pos = int(parts[1])
                right_pos = int(parts[2])
                left_dist = float(parts[3])
                right_dist = float(parts[4])
                
                # Call callback if registered
                if self.odometry_callback:
                    self.odometry_callback(left_pos, right_pos, left_dist, right_dist)
        except Exception as e:
            self.logger.error(f"Failed to process odometry data: {e}")
    
    def _process_debug(self, line):
        """
        Process debug information from Arduino.
        
        Args:
            line: Debug data line from Arduino
        """
        # Call callback if registered
        if self.debug_callback:
            self.debug_callback(line)
        else:
            self.logger.debug(f"Debug: {line}")


# Simple test if run directly
if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Test Arduino serial communication')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--reset', action='store_true', help='Reset port before connecting')
    args = parser.parse_args()
    
    # Reset the port if requested
    if args.reset:
        try:
            import subprocess
            print(f"Attempting to reset port {args.port}...")
            
            # Try to kill any processes using the port
            processes = subprocess.check_output(['fuser', args.port], stderr=subprocess.STDOUT).decode().strip()
            if processes:
                print(f"Found processes using port: {processes}")
                subprocess.run(['fuser', '-k', args.port])
                print("Processes terminated")
            
            # Reset USB device
            if os.path.exists(args.port):
                dev_path = os.path.realpath(args.port)
                if '/dev/ttyACM' in dev_path or '/dev/ttyUSB' in dev_path:
                    bus_dev = dev_path.split('/')[-1]
                    usb_path = f"/sys/bus/usb/devices/{bus_dev.split('tty')[0]}"
                    if os.path.exists(usb_path):
                        with open(f"{usb_path}/power/unbind", 'w') as f:
                            f.write('1')
                        time.sleep(0.5)
                        with open(f"{usb_path}/power/bind", 'w') as f:
                            f.write('1')
                        print(f"Reset USB device at {usb_path}")
        except Exception as e:
            print(f"Error resetting port: {e}")
    
    # Initialize rclpy for logging
    rclpy.init()
    node = Node('arduino_serial_test')
    logger = node.get_logger()
    
    # Create serial interface
    arduino = ImprovedArduinoSerial(port=args.port, baudrate=args.baud, logger=logger)
    
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
            
            # Send debug request periodically
            def debug_timer():
                while rclpy.ok():
                    arduino.send_debug_request()
                    time.sleep(1.0)
            
            debug_thread = threading.Thread(target=debug_timer)
            debug_thread.daemon = True
            debug_thread.start()
            
            # Keep node running
            rclpy.spin(node)
        except KeyboardInterrupt:
            logger.info("Test interrupted")
        finally:
            arduino.disconnect()
            node.destroy_node()
            rclpy.shutdown()
