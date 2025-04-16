#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
# Import transforms3d instead of tf_transformations
from transforms3d.euler import euler2quat

from arduino_motor_bridge.improved_arduino_serial import ImprovedArduinoSerial

class ImprovedArduinoBridge(Node):
    """
    Bridge between ROS2 and Arduino with improved serial communication.
    
    Subscribes to /cmd_vel and sends velocity commands to Arduino.
    Publishes odometry data from Arduino to /odom.
    """
    
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.065)  # meters
        self.declare_parameter('base_width', 0.17)     # meters
        self.declare_parameter('max_linear_speed', 0.5)  # meters/second
        self.declare_parameter('max_angular_speed', 2.0)  # radians/second
        self.declare_parameter('timeout', 0.1)  # seconds
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.base_width = self.get_parameter('base_width').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.timeout = self.get_parameter('timeout').value
        
        self.get_logger().info(f"Serial port: {self.serial_port}")
        self.get_logger().info(f"Wheel radius: {self.wheel_radius} m")
        self.get_logger().info(f"Base width: {self.base_width} m")
        self.get_logger().info(f"Max linear speed: {self.max_linear_speed} m/s")
        self.get_logger().info(f"Max angular speed: {self.max_angular_speed} rad/s")
        
        # Set up QoS profiles
        cmd_vel_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create the serial interface
        self.arduino = ImprovedArduinoSerial(
            port=self.serial_port,
            baudrate=self.baud_rate,
            timeout=self.timeout,
            logger=self.get_logger()
        )
        
        # Connect to the Arduino
        if not self.arduino.connect(retry_count=5):
            self.get_logger().error("Failed to connect to Arduino. Node will continue but will not function properly.")
        
        # Set up Arduino callbacks
        self.arduino.set_odometry_callback(self._handle_odometry)
        self.arduino.set_debug_callback(self._handle_debug)
        
        # Create publishers and subscribers
        self.vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self._handle_cmd_vel,
            cmd_vel_qos
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            odom_qos
        )
        
        # TF broadcaster for odometry
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State variables for odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Timer for safety checks (stop motors if no commands received)
        self.safety_timer = self.create_timer(0.5, self._safety_check)
        
        # Timer for debug info
        self.debug_timer = self.create_timer(5.0, self._request_debug)
        
        # Timer for checking Arduino connection
        self.connection_timer = self.create_timer(1.0, self._check_connection)
        
        self.get_logger().info("Arduino Bridge Node initialized")
    
    def _handle_cmd_vel(self, msg):
        """
        Process incoming velocity commands and convert to wheel velocities.
        
        Args:
            msg: Twist message with linear and angular velocities
        """
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Apply speed limits
        linear_x = max(-self.max_linear_speed, min(linear_x, self.max_linear_speed))
        angular_z = max(-self.max_angular_speed, min(angular_z, self.max_angular_speed))
        
        # Calculate wheel velocities using differential drive kinematics
        left_vel = (linear_x - angular_z * self.base_width / 2.0) / self.wheel_radius
        right_vel = (linear_x + angular_z * self.base_width / 2.0) / self.wheel_radius
        
        # Send velocities to Arduino
        if not self.arduino.send_velocity_command(left_vel, right_vel):
            self.get_logger().warning("Failed to send velocity command")
    
    def _handle_odometry(self, left_pos, right_pos, left_dist, right_dist):
        """
        Process odometry data from Arduino and publish to ROS.
        
        Args:
            left_pos: Left wheel encoder position
            right_pos: Right wheel encoder position
            left_dist: Left wheel distance traveled (m)
            right_dist: Right wheel distance traveled (m)
        """
        # Calculate robot's position
        dist = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / self.base_width
        
        # Update position
        self.x += dist * math.cos(self.theta + delta_theta/2)
        self.y += dist * math.sin(self.theta + delta_theta/2)
        self.theta += delta_theta
        
        # Normalize theta to -pi to pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Current time
        now = self.get_clock().now()
        
        # Publish odometry transform (TF)
        self._publish_odom_tf(now)
        
        # Publish odometry message
        self._publish_odom_msg(now, left_dist, right_dist)
    
    def _publish_odom_tf(self, now):
        """
        Publish transform from odom to base_link.
        
        Args:
            now: Current ROS time
        """
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        # Set transform
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert rotation from yaw to quaternion using transforms3d
        q = euler2quat(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_odom_msg(self, now, left_dist, right_dist):
        """
        Publish odometry message.
        
        Args:
            now: Current ROS time
            left_dist: Left wheel distance traveled
            right_dist: Right wheel distance traveled
        """
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion) using transforms3d
        q = euler2quat(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Set velocity
        # Calculate linear and angular velocities
        if abs(right_dist - left_dist) < 1e-6:
            # Moving straight
            odom.twist.twist.linear.x = (left_dist + right_dist) / 2.0
            odom.twist.twist.angular.z = 0.0
        else:
            # Turning
            odom.twist.twist.linear.x = (left_dist + right_dist) / 2.0
            odom.twist.twist.angular.z = (right_dist - left_dist) / self.base_width
        
        # Publish odometry
        self.odom_pub.publish(odom)
    
    def _handle_debug(self, msg):
        """
        Process debug information from Arduino.
        
        Args:
            msg: Debug message from Arduino
        """
        self.get_logger().info(f"Arduino Debug: {msg}")
    
    def _safety_check(self):
        """
        Stop motors if no commands received recently.
        """
        now = self.get_clock().now()
        dt = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        
        if dt > 1.0:  # 1 second timeout
            # Send zero velocity to stop the robot
            if self.arduino.connected:
                self.arduino.send_velocity_command(0.0, 0.0)
    
    def _request_debug(self):
        """
        Request debug information from Arduino periodically.
        """
        if self.arduino.connected:
            self.arduino.send_debug_request()
    
    def _check_connection(self):
        """
        Check Arduino connection status and attempt reconnection if needed.
        """
        if not self.arduino.connected:
            self.get_logger().warn("Arduino disconnected, attempting to reconnect...")
            self.arduino.connect(retry_count=1)
    
    def shutdown(self):
        """
        Perform shutdown cleanup.
        """
        self.get_logger().info("Shutting down Arduino Bridge Node")
        
        # Send zero velocity to stop the robot
        if self.arduino.connected:
            self.arduino.send_velocity_command(0.0, 0.0)
        
        # Disconnect from Arduino
        self.arduino.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = ImprovedArduinoBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Uncaught exception: {e}")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
