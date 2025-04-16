#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class TrackRotationSimulator(Node):
    """
    This node simulates track rotation based on velocity commands.
    It subscribes to velocity commands and publishes joint states to make
    the wheels rotate accordingly, creating the visual effect of track movement.
    """

    def __init__(self):
        super().__init__('track_rotation_simulator')
        
        # Parameters
        self.declare_parameter('wheel_radius', 0.095)  # meters (default: 190mm diameter / 2)
        self.declare_parameter('update_rate', 30.0)  # Hz
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Track joint names - must match URDF
        self.joint_names = [
            'left_front_wheel_joint',
            'left_rear_wheel_joint',
            'right_front_wheel_joint',
            'right_rear_wheel_joint'
        ]
        
        # Joint state publisher
        self.joint_state_pub = self.create_publisher(JointState, 'wheel_joint_states', 10)
        
        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscribe to odometry to get actual velocities
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Current wheel positions
        self.wheel_positions = [0.0] * len(self.joint_names)
        
        # Current velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Timer for publishing joint states
        self.last_update_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_joint_states)
        
        self.get_logger().info('Track rotation simulator started')

    def cmd_vel_callback(self, msg):
        """Process velocity commands"""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def odom_callback(self, msg):
        """Process odometry information (if available)"""
        # If we're getting odometry, use it instead of cmd_vel
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

    def update_joint_states(self):
        """Update joint positions based on current velocities"""
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9  # Convert to seconds
        self.last_update_time = now
        
        # Calculate wheel velocities from robot velocities
        # For differential drive with tracks:
        # - Left and right wheels on the same side move at the same rate
        # - Wheels on opposite sides may move at different rates for turning
        track_separation = 0.644  # meters (from URDF: robot_width - track_width)
        
        # Calculate track velocities (linear velocity at each track)
        left_track_vel = self.linear_velocity - (self.angular_velocity * track_separation / 2.0)
        right_track_vel = self.linear_velocity + (self.angular_velocity * track_separation / 2.0)
        
        # Calculate angular wheel velocities
        left_wheel_angular_vel = left_track_vel / self.wheel_radius
        right_wheel_angular_vel = right_track_vel / self.wheel_radius
        
        # Update wheel positions
        self.wheel_positions[0] += left_wheel_angular_vel * dt   # left front
        self.wheel_positions[1] += left_wheel_angular_vel * dt   # left rear
        self.wheel_positions[2] += right_wheel_angular_vel * dt  # right front
        self.wheel_positions[3] += right_wheel_angular_vel * dt  # right rear
        
        # Normalize angles to -pi to pi (optional)
        for i in range(len(self.wheel_positions)):
            self.wheel_positions[i] = (self.wheel_positions[i] + math.pi) % (2 * math.pi) - math.pi
        
        # Create and publish joint state message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = now.to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.wheel_positions
        joint_state_msg.velocity = [left_wheel_angular_vel, left_wheel_angular_vel, 
                                   right_wheel_angular_vel, right_wheel_angular_vel]
        joint_state_msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_state_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackRotationSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
