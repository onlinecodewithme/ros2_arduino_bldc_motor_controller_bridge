#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat

from arduino_motor_bridge.robust_arduino_serial import RobustArduinoSerial

class RobustEncoderNode(Node):
    """
    High reliability E38S encoder node with improved serial connection management.
    
    This specialized node provides more accurate odometry based on the
    E38S 1000B G24N encoders with advanced port locking and recovery features.
    """

    def __init__(self):
        super().__init__('robust_encoder_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.0825)  # meters (adjusted for your robot)
        self.declare_parameter('base_width', 0.17)      # meters (distance between wheels)
        self.declare_parameter('timeout', 0.1)          # seconds
        self.declare_parameter('publish_rate', 20.0)    # Hz
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('encoder_ticks_per_rev', 4000.0)  # For E38S encoders with quadrature
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 2.0) # rad/s

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.base_width = self.get_parameter('base_width').value
        self.timeout = self.get_parameter('timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.encoder_ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        # Calculate meters per tick for the encoder
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.encoder_ticks_per_rev
        
        self.get_logger().info(f"Using E38S encoders with {self.encoder_ticks_per_rev} ticks per revolution")
        self.get_logger().info(f"Wheel radius: {self.wheel_radius} m")
        self.get_logger().info(f"Base width: {self.base_width} m")
        self.get_logger().info(f"Meters per tick: {self.meters_per_tick} m")
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

        # Create the serial interface with robust error handling
        self.arduino = RobustArduinoSerial(
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
            self.cmd_vel_topic,
            self._handle_cmd_vel,
            cmd_vel_qos
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            odom_qos
        )

        # TF broadcaster for odometry
        self.tf_broadcaster = TransformBroadcaster(self)

        # State variables for odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_dist = 0.0
        self.last_right_dist = 0.0
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Velocity estimation
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_odom_time = self.get_clock().now()

        # Timer for safety checks (stop motors if no commands received)
        self.safety_timer = self.create_timer(0.5, self._safety_check)

        # Timer for debug info
        self.debug_timer = self.create_timer(5.0, self._request_debug)

        # Timer for checking Arduino connection
        self.connection_timer = self.create_timer(1.0, self._check_connection)

        # Reset odometry at startup
        self.arduino.send_reset_command()

        # Timer for publishing odometry at a fixed rate (in addition to the updates from Arduino)
        self.odom_timer = self.create_timer(1.0/self.publish_rate, self._publish_odometry)

        self.get_logger().info("Robust E38S Encoder Node initialized")

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

        # Store current velocity for odometry estimation
        self.linear_velocity = linear_x
        self.angular_velocity = angular_z

        # Apply speed limits
        linear_x = max(-self.max_linear_speed, min(linear_x, self.max_linear_speed))
        angular_z = max(-self.max_angular_speed, min(angular_z, self.max_angular_speed))

        # Calculate wheel velocities using differential drive kinematics
        left_vel = (linear_x - angular_z * self.base_width / 2.0) / self.wheel_radius
        right_vel = (linear_x + angular_z * self.base_width / 2.0) / self.wheel_radius

        # Send velocities to Arduino with robust error handling
        if not self.arduino.send_velocity_command(left_vel, right_vel):
            self.get_logger().warning("Failed to send velocity command")

    def _handle_odometry(self, left_pos, right_pos, left_dist, right_dist):
        """
        Process odometry data from Arduino and update robot position.

        Args:
            left_pos: Left wheel encoder position in ticks
            right_pos: Right wheel encoder position in ticks
            left_dist: Left wheel distance traveled in meters
            right_dist: Right wheel distance traveled in meters
        """
        # Calculate distance traveled since last update
        delta_left = left_dist - self.last_left_dist
        delta_right = right_dist - self.last_right_dist
        
        # Save current distances for next calculation
        self.last_left_dist = left_dist
        self.last_right_dist = right_dist
        
        # Skip tiny movements (noise reduction)
        if abs(delta_left) < 0.0001 and abs(delta_right) < 0.0001:
            return
            
        # Calculate robot's position change
        delta_dist = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.base_width
        
        # Skip unreasonable jumps (error protection)
        if abs(delta_dist) > 0.1 or abs(delta_theta) > 0.2:
            self.get_logger().warning(f"Ignoring unreasonable odometry jump: dist={delta_dist}, theta={delta_theta}")
            return

        # Update position using improved integration method
        if abs(delta_theta) < 0.0001:
            # Moving straight
            self.x += delta_dist * math.cos(self.theta)
            self.y += delta_dist * math.sin(self.theta)
        else:
            # Moving in an arc
            radius = delta_dist / delta_theta
            theta_mid = self.theta + delta_theta / 2.0
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
            self.theta += delta_theta

        # Normalize theta to -pi to pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Get current time for publishing
        now = self.get_clock().now()
        
        # Calculate time delta for velocity estimation
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        
        # Estimate velocities if time delta is reasonable
        if dt > 0.001:
            # Linear and angular velocities from encoder changes
            self.linear_velocity = delta_dist / dt
            self.angular_velocity = delta_theta / dt

        # Publish odometry
        self._publish_odometry_with_time(now)

    def _publish_odometry(self):
        """
        Publish odometry data at a fixed rate.
        """
        now = self.get_clock().now()
        self._publish_odometry_with_time(now)

    def _publish_odometry_with_time(self, now):
        """
        Publish odometry transform and message.

        Args:
            now: Current ROS time
        """
        # Publish odometry transform (TF)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

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

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Set orientation (quaternion)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Add some uncertainty to the pose
        odom.pose.covariance[0] = 0.001  # x
        odom.pose.covariance[7] = 0.001  # y
        odom.pose.covariance[35] = 0.01  # yaw

        # Set velocity in robot's frame
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity

        # Add some uncertainty to the twist
        odom.twist.covariance[0] = 0.001  # vx
        odom.twist.covariance[35] = 0.01  # vyaw

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
                if self.linear_velocity != 0.0 or self.angular_velocity != 0.0:
                    self.linear_velocity = 0.0
                    self.angular_velocity = 0.0
                    self.get_logger().debug("Safety stop: no commands received")

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
            self.arduino.connect(retry_count=5)

    def shutdown(self):
        """
        Perform shutdown cleanup.
        """
        self.get_logger().info("Shutting down Robust Encoder Node")

        # Send zero velocity to stop the robot
        if self.arduino.connected:
            self.arduino.send_velocity_command(0.0, 0.0)

        # Disconnect from Arduino
        self.arduino.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = RobustEncoderNode()

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
