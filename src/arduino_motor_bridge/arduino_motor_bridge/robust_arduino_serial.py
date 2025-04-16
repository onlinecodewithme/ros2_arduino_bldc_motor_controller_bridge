#!/usr/bin/env python3

import struct
import time
import threading
import serial
import rclpy
import subprocess
import os
import fcntl
import errno
from rclpy.logging import get_logger

class RobustArduinoSerial:
    """
    Highly robust serial communication with Arduino for E38S encoder integration.
    Features improved port locking, automatic recovery, and process management.
    """

    # Protocol constants
    COMMAND_HEADER = b'#'
    VELOCITY_HEADER = b'V'
    ODOMETRY_HEADER = b'O'
    RESET_HEADER = b'R'
    ACK_HEADER = b'A'
    DEBUG_HEADER = b'D'

    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=0.1, logger=None):
        """
        Initialize serial connection with advanced error recovery.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.connected = False
        self.last_error_time = 0
        self.error_count = 0
        self.logger = logger or get_logger('robust_arduino_serial')
        self.read_thread = None
        self.running = False
        self.reconnect_thread = None
        self.is_reconnecting = False
        self.odometry_callback = None
        self.debug_callback = None
        self.lock = threading.RLock()  # Reentrant lock for nested operations
        self.port_lock_file = None

    def connect(self, retry_count=5, retry_delay=2.0):
        """
        Establish serial connection with robust error handling.
        """
        # First check if another process is using the port
        self._kill_competing_processes()
        
        for attempt in range(retry_count):
            try:
                self.logger.info(f"Connection attempt {attempt+1}/{retry_count} to Arduino on {self.port}")

                # Close any existing connection
                self._close_existing_connection()
                
                # Wait before retry
                if attempt > 0:
                    time.sleep(retry_delay)
                
                # Try to lock the port with a lockfile
                if not self._lock_port():
                    self.logger.error(f"Could not acquire exclusive lock for {self.port}")
                    continue

                # Create new serial connection
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    write_timeout=1.0,
                    exclusive=True  # Also use pyserial's exclusive mode
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
                
                # Special handling for "device busy" errors - try to kill competing processes
                if "could not exclusively lock" in str(e) or "Resource temporarily unavailable" in str(e):
                    self._kill_competing_processes()
                    time.sleep(1.0)  # Wait a moment for the port to free up

            except Exception as e:
                self.logger.error(f"Unexpected error on attempt {attempt+1}: {e}")
                self._close_existing_connection()

        self.logger.error(f"Failed to connect to Arduino after {retry_count} attempts")
        self.connected = False
        return False

    def _lock_port(self):
        """
        Create a lockfile to ensure exclusive access to the port.
        """
        try:
            # Create lockfile
            lockfile_path = f"/tmp/arduino_port_{os.path.basename(self.port)}.lock"
            self.port_lock_file = open(lockfile_path, 'w')
            
            # Try non-blocking exclusive lock
            try:
                fcntl.flock(self.port_lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self.logger.info(f"Acquired lock file for port {self.port}")
                return True
            except IOError as e:
                if e.errno == errno.EACCES or e.errno == errno.EAGAIN:
                    self.logger.error(f"Port {self.port} is locked by another process")
                    self.port_lock_file.close()
                    self.port_lock_file = None
                    return False
                raise
                
        except Exception as e:
            self.logger.error(f"Error creating lock file: {e}")
            if self.port_lock_file:
                self.port_lock_file.close()
                self.port_lock_file = None
            return False
    
    def _release_port_lock(self):
        """
        Release the lockfile.
        """
        if self.port_lock_file:
            try:
                fcntl.flock(self.port_lock_file, fcntl.LOCK_UN)
                self.port_lock_file.close()
            except Exception as e:
                self.logger.warning(f"Error releasing port lock: {e}")
            self.port_lock_file = None

    def _kill_competing_processes(self):
        """
        Find and kill any processes that might be using the port.
        """
        try:
            self.logger.info(f"Checking for processes using port {self.port}")
            result = subprocess.run(['fuser', self.port], 
                                  stdout=subprocess.PIPE, 
                                  stderr=subprocess.PIPE,
                                  text=True)
            if result.stdout.strip():
                pids = result.stdout.strip().split()
                self.logger.warning(f"Found processes using port {self.port}: {pids}")
                
                # Kill the processes
                subprocess.run(['fuser', '-k', self.port], 
                             stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE)
                self.logger.info(f"Killed processes using port {self.port}")
                time.sleep(1.0)  # Wait for processes to terminate
                return True
                
        except Exception as e:
            self.logger.warning(f"Error checking/killing processes: {e}")
        
        return False

    def _close_existing_connection(self):
        """
        Safely close existing serial connection and release locks.
        """
        with self.lock:
            try:
                if self.serial and self.serial.is_open:
                    self.serial.close()
                    self.logger.info("Closed existing serial connection")
            except Exception as e:
                self.logger.warning(f"Error closing existing connection: {e}")

            # Release port lock
            self._release_port_lock()
            
            # Clear serial object
            self.serial = None
            self.connected = False

    def _reset_arduino(self):
        """
        Reset Arduino by toggling DTR line with proper timing.
        """
        with self.lock:
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
        """
        with self.lock:
            try:
                self.logger.info("Testing Arduino connection...")

                # Send test command
                self.serial.write(self.COMMAND_HEADER + b'?')
                time.sleep(0.5)

                # Read response
                max_wait = 3.0  # seconds
                start_time = time.time()
                
                while (time.time() - start_time) < max_wait:
                    bytes_waiting = self.serial.in_waiting
                    if bytes_waiting > 0:
                        response = self.serial.read(bytes_waiting)
                        self.logger.info(f"Arduino test response length: {len(response)}")
                        return len(response) > 0
                    time.sleep(0.1)
                    
                self.logger.warning("Timeout waiting for Arduino test response")
                return False
                
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
                self.logger.debug(f"Sent velocity command: L={left_vel:.2f} R={right_vel:.2f}")
                return True
        except Exception as e:
            self.logger.error(f"Failed to send velocity command: {e}")
            self._handle_error()
            return False

    def send_reset_command(self):
        """
        Send reset command to reset odometry counters.
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
        """
        self.odometry_callback = callback

    def set_debug_callback(self, callback):
        """
        Set callback for debug information.
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

            # First check and kill any processes that might be using the port
            self._kill_competing_processes()
            
            # Wait a bit before attempting reconnection
            time.sleep(2.0)

            # Close any existing connection
            self._close_existing_connection()

            # Try to connect with retries
            if self.connect(retry_count=5, retry_delay=1.0):
                self.logger.info("Reconnected successfully to Arduino")
            else:
                self.logger.error("Failed to reconnect to Arduino")
                
                # If we still can't connect, try a more aggressive approach
                self.logger.warning("Attempting more aggressive port recovery...")
                self._reset_port_aggressively()
                
                # Try one more time
                if self.connect(retry_count=3, retry_delay=1.0):
                    self.logger.info("Reconnected successfully after aggressive recovery")
                else:
                    self.logger.error("Failed to reconnect even after aggressive recovery")
                
        except Exception as e:
            self.logger.error(f"Error in reconnection thread: {e}")
        finally:
            self.is_reconnecting = False
            
    def _reset_port_aggressively(self):
        """
        More aggressive port recovery when standard methods fail
        """
        try:
            # Try to reset the USB device
            self.logger.warning(f"Attempting to reset USB device for {self.port}")
            
            # 1. Kill ALL processes that might be accessing any serial port
            subprocess.run(['fuser', '-k', '/dev/ttyACM*'], 
                          stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE)
            
            # 2. Try to unbind and rebind the USB device if we can find it
            if os.path.exists(self.port):
                # Get the USB bus and device number
                dev_path = os.path.realpath(self.port)
                if '/dev/ttyACM' in dev_path:
                    # Simplified reset - just wait a bit to let the system recover
                    time.sleep(5.0)
                    
            self.logger.info("Completed aggressive port recovery")
            
        except Exception as e:
            self.logger.error(f"Error during aggressive port reset: {e}")

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

                if self.serial and self.serial.in_waiting:
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
                if consecutive_errors > 5:
                    self.logger.error("Too many consecutive errors, marking connection as lost")
                    self.connected = False
                    self._handle_error()

                # Sleep longer to avoid rapid error logging
                time.sleep(0.1)

        self.logger.info("Serial read thread stopped")

    def _process_odometry(self, line):
        """
        Process odometry data from Arduino.
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
            subprocess.run(['fuser', '-k', args.port], 
                          stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE)
            print("Port reset completed")

        except Exception as e:
            print(f"Error resetting port: {e}")

    # Initialize rclpy for logging
    rclpy.init()
    node = Node('arduino_serial_test')
    logger = node.get_logger()

    # Create serial interface
    arduino = RobustArduinoSerial(port=args.port, baudrate=args.baud, logger=logger)

    # Connect to Arduino
    if arduino.connect():
        try:
            # Set up callbacks
            arduino.set_odometry_callback(lambda l, r, ld, rd:
                logger.info(f"Odometry: left={l} right={r} left_dist={ld:.3f} right_dist={rd:.3f}"))

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
