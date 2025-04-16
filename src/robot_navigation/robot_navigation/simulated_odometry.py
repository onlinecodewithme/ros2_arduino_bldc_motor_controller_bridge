#!/usr/bin/env python3

"""
Simulated Odometry Publisher

This node publishes fake odometry data to simulate a moving robot without
requiring actual hall sensor feedback. This allows for visualization in RViz
and testing of the navigation stack with simulated movement.

Usage:
  ros2 run robot_navigation simulated_odometry
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import time
import threading
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat


class SimulatedOdometry(Node):
    """
    Publishes simulated odometry data to mimic a mobile robot's movement.

    This node:
    1. Subscribes to cmd_vel to know desired movement
    2. Simulates realistic robot motion with acceleration limits
    3. Publishes odometry data to /odom topic
    4. Broadcasts tf transforms
    """

    def __init__(self):
        super().__init__('simulated_odometry')

        # Declare parameters
        self.declare_parameter('wheel_radius', 0.065)  # meters
        self.declare_parameter('base_width', 0.17)     # meters
        self.declare_parameter('publish_rate', 20.0)   # Hz
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s
        self.declare_parameter('acceleration', 0.5)    # m/s²
        self.declare_parameter('angular_acceleration', 1.0)  # rad/s²

        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.base_width = self.get_parameter('base_width').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.acceleration = self.get_parameter('acceleration').value
        self.angular_acceleration = self.get_parameter('angular_acceleration').value

        # Set up QoS profiles
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',  # Standard odometry topic
            odom_qos
        )

        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Current velocity
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Target velocity (from cmd_vel)
        self.target_linear = 0.0
        self.target_angular = 0.0

        self.lock = threading.Lock()

        # Create timer for publishing odometry
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odometry)

        self.get_logger().info('Simulated Odometry node initialized')

    def cmd_vel_callback(self, msg):
        """Process velocity commands."""
        with self.lock:
            self.target_linear = msg.linear.x
            self.target_angular = msg.angular.z

            # Clamp to max speeds
            self.target_linear = max(-self.max_linear_speed,
                                     min(self.max_linear_speed, self.target_linear))
            self.target_angular = max(-self.max_angular_speed,
                                      min(self.max_angular_speed, self.target_angular))

    def publish_odometry(self):
        """Calculate and publish odometry data based on simulated motion."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt < 0.0001:
            return  # Avoid division by zero

        with self.lock:
            # Simulate acceleration limits
            if self.current_linear < self.target_linear:
                self.current_linear = min(self.target_linear,
                                         self.current_linear + self.acceleration * dt)
            elif self.current_linear > self.target_linear:
                self.current_linear = max(self.target_linear,
                                         self.current_linear - self.acceleration * dt)

            if self.current_angular < self.target_angular:
                self.current_angular = min(self.target_angular,
                                          self.current_angular + self.angular_acceleration * dt)
            elif self.current_angular > self.target_angular:
                self.current_angular = max(self.target_angular,
                                          self.current_angular - self.angular_acceleration * dt)

            # Simulate robot movement
            delta_x = self.current_linear * dt * math.cos(self.theta)
            delta_y = self.current_linear * dt * math.sin(self.theta)
            delta_theta = self.current_angular * dt

            # Update robot position
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # Normalize theta to -pi to pi
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            # Log position occasionally
            if int(now.nanoseconds / 1e9) % 5 == 0:
                self.get_logger().debug(f"Position: ({self.x:.2f}, {self.y:.2f}, {self.theta:.2f})")

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
        odom.twist.twist.linear.x = self.current_linear
        odom.twist.twist.angular.z = self.current_angular

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


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
