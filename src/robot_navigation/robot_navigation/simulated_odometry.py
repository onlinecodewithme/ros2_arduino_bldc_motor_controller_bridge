#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time
from builtin_interfaces.msg import Time


class SimulatedOdometry(Node):
    """
    This node simulates odometry for the tracked robot.
    It publishes a transform from odom to base_link and an odometry message.
    """

    def __init__(self):
        super().__init__('simulated_odometry')
        
        # Parameters
        self.declare_parameter('update_rate', 30.0)  # Hz
        self.update_rate = self.get_parameter('update_rate').value
        
        # Robot's current position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Robot's current velocity
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # Last update time
        self.last_update_time = self.get_clock().now()
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Subscribe to cmd_vel to get velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Timer for updating odometry
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_odometry)
        
        self.get_logger().info('Simulated odometry started')

    def cmd_vel_callback(self, msg):
        """Process velocity commands"""
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def update_odometry(self):
        """Update odometry and publish transform"""
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9  # Convert to seconds
        self.last_update_time = now
        
        # Update robot's position based on velocity
        if abs(self.angular_z) < 0.0001:  # Going straight
            self.x += self.linear_x * dt * math.cos(self.theta)
            self.y += self.linear_x * dt * math.sin(self.theta)
        else:  # Following an arc
            radius = self.linear_x / self.angular_z
            self.x += radius * (math.sin(self.theta + self.angular_z * dt) - math.sin(self.theta))
            self.y += radius * (math.cos(self.theta) - math.cos(self.theta + self.angular_z * dt))
            self.theta += self.angular_z * dt
            
            # Normalize angle to -pi to pi
            self.theta = ((self.theta + math.pi) % (2 * math.pi)) - math.pi
        
        # Create and publish transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert euler angle to quaternion
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)
        
        # Create and publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        
        # Set velocity
        odom.twist.twist.linear.x = self.linear_x
        odom.twist.twist.angular.z = self.angular_z
        
        # Publish odometry
        self.odom_pub.publish(odom)


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
