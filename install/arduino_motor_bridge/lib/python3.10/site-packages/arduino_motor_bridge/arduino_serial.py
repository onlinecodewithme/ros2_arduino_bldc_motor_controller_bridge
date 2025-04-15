#!/usr/bin/env python3

import struct
import time
import threading
import serial
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

class ArduinoSerial:
    """
    Handles serial communication with the Arduino motor controller.
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
        Initialize serial connection to Arduino.
        
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
        self.logger = logger or get_logger('arduino_serial')
        self.read_thread = None
        self.running = False
        self.odometry_callback = None
        self.debug_callback = None
        self.lock = threading.Lock()
        
    def connect(self):
        """
        Establish serial connection to Arduino.
        """
        try:
            self.logger.info(f"Attempting to connect to Arduino on {self.port}")
            # First try to close if already open
            try:
                if self.serial and self.serial.is_open:
                    self.serial.close()
                    self.logger.info("Closed existing serial connection")
            except:
                pass
                
            # Clear any existing serial object
            self.serial = None
            
            # Wait a moment
            time.sleep(1.0)
            
            # Create new serial connection
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=1.0,
                exclusive=False  # Don't lock the port
            )
            
            # Reset Arduino by toggling DTR line
            self.logger.info("Toggling DTR line to reset Arduino...")
            self.serial.dtr = False
            time.sleep(0.5)
            self.serial.dtr = True
            time.sleep(2.0)  # Wait for Arduino to initialize
            
            # Flush initial data
            self.logger.info("Flushing buffers...")
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # Send a test command to see if Arduino responds
            self.logger.info("Sending test command...")
            self.serial.write(self.COMMAND_HEADER + b'?')
            time.sleep(0.5)
            
            # Try to read some data
            response = self.serial.read(self.serial.in_waiting or 1)
            self.logger.info(f"Arduino test response length: {len(response)}")
            
            self.connected = True
            self.logger.info(f"Connected to Arduino on {self.port}")
            
            # Start the reading thread
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop)
            self.read_thread.daemon = True
            self.read_thread.start()
            return True
        except serial.SerialException as e:
            self.logger.error(f"Failed to connect to Arduino: {e}")
            self.connected = False
            return False
        except Exception as e:
            self.logger.error(f"Unexpected error connecting to Arduino: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """
        Close serial connection.
        """
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.logger.info("Disconnected from Arduino")
        
        self.connected = False
    
    def send_velocity_command(self, left_vel, right_vel):
        """
        Send velocity command to Arduino.
        
        Args:
            left_vel: Left wheel velocity in m/s
            right_vel: Right wheel velocity in m/s
        
        Returns:
            bool: True if command sent successfully
        """
        if not self.connected:
            self.logger.warning("Not connected to Arduino")
            return False
        
        try:
            with self.lock:
                # Pack two floats into binary format
                left_bytes = struct.pack('<f', left_vel)
                right_bytes = struct.pack('<f', right_vel)
                
                # Send command: VELOCITY_HEADER + left_bytes + right_bytes
                self.serial.write(self.VELOCITY_HEADER + left_bytes + right_bytes)
                self.logger.debug(f"Sent velocity command: L={left_vel}, R={right_vel}")
                return True
        except Exception as e:
            self.logger.error(f"Failed to send velocity command: {e}")
            return False
    
    def send_reset_command(self):
        """
        Send reset command to reset odometry counters.
        
        Returns:
            bool: True if command sent successfully
        """
        if not self.connected:
            self.logger.warning("Not connected to Arduino")
            return False
        
        try:
            with self.lock:
                self.serial.write(self.RESET_HEADER)
                self.logger.debug("Sent reset command")
                return True
        except Exception as e:
            self.logger.error(f"Failed to send reset command: {e}")
            return False
    
    def send_debug_request(self):
        """
        Request debug information from Arduino.
        
        Returns:
            bool: True if request sent successfully
        """
        if not self.connected:
            self.logger.warning("Not connected to Arduino")
            return False
        
        try:
            with self.lock:
                self.serial.write(self.DEBUG_HEADER)
                self.logger.debug("Sent debug request")
                return True
        except Exception as e:
            self.logger.error(f"Failed to send debug request: {e}")
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
            self.logger.warning("Not connected to Arduino")
            return False
        
        try:
            with self.lock:
                self.serial.write(self.COMMAND_HEADER + command_char.encode('ascii'))
                self.logger.debug(f"Sent manual command: {command_char}")
                return True
        except Exception as e:
            self.logger.error(f"Failed to send manual command: {e}")
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
    
    def _read_loop(self):
        """
        Background thread for reading serial data.
        """
        self.logger.info("Serial read thread started")
        
        while self.running:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('ascii', errors='ignore').strip()
                    
                    if not line:
                        continue
                    
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
                    time.sleep(0.001)
            except Exception as e:
                self.logger.error(f"Error in serial read thread: {e}")
                # If we encounter a serious error, sleep longer to avoid rapid error logging
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
    args = parser.parse_args()
    
    # Initialize rclpy for logging
    rclpy.init()
    node = Node('arduino_serial_test')
    logger = node.get_logger()
    
    # Create serial interface
    arduino = ArduinoSerial(port=args.port, baudrate=args.baud, logger=logger)
    
    # Connect to Arduino
    if arduino.connect():
        try:
            # Set up callbacks
            arduino.set_odometry_callback(lambda l, r, ld, rd: 
                logger.info(f"Odometry: left={l}, right={r}, left_dist={ld}, right_dist={rd}"))
            
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
