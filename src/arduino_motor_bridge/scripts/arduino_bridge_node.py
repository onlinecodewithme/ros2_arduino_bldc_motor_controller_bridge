#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import tf2_ros

from arduino_motor_bridge.arduino_serial import ArduinoSerial

class ArduinoBridgeNode(Node):
    """
    ROS2 node for bridging between ROS2 and Arduino motor controller.
    
    Subscribes to:
    - cmd_vel (Twist): Velocity commands
    
    Publishes:
    - odom (Odometry): Odometry information
    - tf: Odometry transform
    
    Services:
    - reset_odom: Reset odometry to zero
    """
    
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Declare and get parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0', 
                              ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                 description='Serial port for Arduino'))
        self.declare_parameter('baud_rate', 115200, 
                              ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                                                 description='Baud rate for serial communication'))
        self.declare_parameter('wheel_separation', 0.35, 
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Distance between wheels in meters'))
        self.declare_parameter('wheel_radius', 0.0825, 
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Wheel radius in meters'))
        self.declare_parameter('use_tf', True, 
                              ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                                 description='Whether to publish TF transform'))
        self.declare_parameter('odom_frame', 'odom', 
                              ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                 description='Odometry frame ID'))
        self.declare_parameter('base_frame', 'base_link', 
                              ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                 description='Base frame ID'))
        self.declare_parameter('publish_rate', 10.0, 
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Rate at which to publish odometry'))
        
        # Get parameter values
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.use_tf = self.get_parameter('use_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Create publishers and subscribers
        self.vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )
        
        # Create transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create services
        self.reset_service = self.create_service(
            Trigger,
            'reset_odom',
            self.reset_odom_callback
        )
        
        # Initialize Arduino serial connection
        self.arduino = ArduinoSerial(
            port=self.serial_port,
            baudrate=self.baud_rate,
            logger=self.get_logger()
        )
        
        # Set up Arduino callbacks
        self.arduino.set_odometry_callback(self.odometry_callback)
        
        # Initialize odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_dist = 0.0
        self.last_right_dist = 0.0
        
        # Try to connect to Arduino
        if not self.arduino.connect():
            self.get_logger().error("Failed to connect to Arduino. Please check connections and try again.")
            # We'll continue running to allow the user to fix the connection
        
        # Create a timer for publishing odometry
        self.odom_timer = self.create_timer(1.0 / self.publish_rate, self.publish_odometry)
        
        self.get_logger().info("Arduino Bridge Node initialized")
    
    def velocity_callback(self, msg: Twist):
        """
        Process velocity commands from ROS2 and send to Arduino.
        
        Args:
            msg: Twist message
        """
        # Convert twist to differential drive commands
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Calculate wheel velocities
        # v_l = linear_x - (angular_z * wheel_separation / 2)
        # v_r = linear_x + (angular_z * wheel_separation / 2)
        left_vel = linear_x - (angular_z * self.wheel_separation / 2)
        right_vel = linear_x + (angular_z * self.wheel_separation / 2)
        
        # Send to Arduino
        self.arduino.send_velocity_command(left_vel, right_vel)
    
    def odometry_callback(self, left_pos: int, right_pos: int, left_dist: float, right_dist: float):
        """
        Process odometry data from Arduino.
        
        Args:
            left_pos: Left wheel encoder position
            right_pos: Right wheel encoder position
            left_dist: Left wheel distance in meters
            right_dist: Right wheel distance in meters
        """
        # Calculate distance traveled and heading change since last update
        delta_left = left_dist - self.last_left_dist
        delta_right = right_dist - self.last_right_dist
        
        # Update stored distances
        self.last_left_dist = left_dist
        self.last_right_dist = right_dist
        
        # Calculate robot's movement
        delta_dist = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_separation
        
        # Update pose estimate (simple differential drive kinematics)
        # Assuming small enough delta_theta that we can use a simple approximation
        self.theta += delta_theta
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Approximation for small angles, more accurate would use integration
        self.x += delta_dist * math.cos(self.theta)
        self.y += delta_dist * math.sin(self.theta)
    
    def publish_odometry(self):
        """
        Publish odometry information.
        """
        current_time = self.get_clock().now().to_msg()
        
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion from yaw)
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = sy
        odom_msg.pose.pose.orientation.w = cy
        
        # Velocity is calculated from the commanded velocity, not from encoders
        # This is an approximation
        # For more accurate velocity, we would need to calculate it from position changes
        
        # Publish odometry message
        self.odom_pub.publish(odom_msg)
        
        # Publish transform if enabled
        if self.use_tf:
            self.publish_tf(current_time)
    
    def publish_tf(self, timestamp):
        """
        Publish TF transform from odom frame to base frame.
        
        Args:
            timestamp: Current timestamp
        """
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        # Set translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Set rotation (quaternion from yaw)
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)
    
    def reset_odom_callback(self, request, response):
        """
        Service callback to reset odometry.
        
        Args:
            request: Service request
            response: Service response
        
        Returns:
            response: Service response
        """
        # Reset odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_dist = 0.0
        self.last_right_dist = 0.0
        
        # Reset Arduino odometry
        self.arduino.send_reset_command()
        
        response.success = True
        response.message = "Odometry reset to zero"
        return response
    
    def on_shutdown(self):
        """
        Clean shutdown when the node is terminated.
        """
        self.get_logger().info("Shutting down Arduino Bridge Node")
        
        # Disconnect from Arduino
        if self.arduino:
            # Stop the robot
            self.arduino.send_velocity_command(0.0, 0.0)
            self.arduino.disconnect()

def main(args=None):
    rclpy.init(args=args)
    
    node = ArduinoBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
