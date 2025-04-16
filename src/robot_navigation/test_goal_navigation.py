#!/usr/bin/env python3

import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from action_msgs.msg import GoalStatus

class RobotNavigationTest(Node):
    """
    Test node for robot navigation.
    
    This node sends a navigation goal to the robot, monitors velocity commands,
    and tracks the navigation progress.
    """
    
    def __init__(self):
        super().__init__('robot_navigation_test')
        
        # Set up velocity subscriber (to monitor what's being sent to Arduino)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._handle_cmd_vel,
            10
        )
        
        # Set up action client for navigation
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Set up laser scan subscriber to detect obstacles
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._handle_scan,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
        )
        
        # Variables
        self.goal_sent = False
        self.goal_reached = False
        self.last_cmd_vel = None
        self.last_scan = None
        self.start_time = None
        
        # Create a simple timer to check goal status
        self.timer = self.create_timer(1.0, self._check_status)
        
        self.get_logger().info('=== Robot Navigation Test Initialized ===')
        self.get_logger().info('Waiting for navigation action server...')
        
    def _handle_cmd_vel(self, msg):
        """Handle command velocity messages."""
        self.last_cmd_vel = msg
        
        # Only log when values change significantly
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.get_logger().info(f'Motor Command - Linear: {msg.linear.x:.3f} m/s, Angular: {msg.angular.z:.3f} rad/s')
    
    def _handle_scan(self, msg):
        """Handle laser scan messages for obstacle detection."""
        self.last_scan = msg
        
        # Check for potential obstacles
        min_distance = float('inf')
        for i, range_val in enumerate(msg.ranges):
            if range_val < min_distance and range_val > msg.range_min:
                min_distance = range_val
                angle = msg.angle_min + i * msg.angle_increment
        
        if min_distance < 0.5:  # Less than 0.5m is considered an obstacle
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m')
    
    def _check_status(self):
        """Periodically check navigation status."""
        if self.goal_sent and not self.goal_reached:
            elapsed = time.time() - self.start_time
            self.get_logger().info(f'Navigation in progress... ({elapsed:.1f}s)')
            
            # Check for movement
            if self.last_cmd_vel:
                if abs(self.last_cmd_vel.linear.x) < 0.01 and abs(self.last_cmd_vel.angular.z) < 0.01:
                    self.get_logger().info('Robot is currently stopped')
                else:
                    if abs(self.last_cmd_vel.linear.x) > abs(self.last_cmd_vel.angular.z):
                        self.get_logger().info('Robot is moving forward/backward')
                    else:
                        self.get_logger().info('Robot is turning')
    
    def send_goal(self, x, y, theta=0.0):
        """Send a navigation goal to the action server."""
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        
        # Set the goal pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion (simple yaw rotation)
        cy = math.cos(theta * 0.5)
        sy = math.sin(theta * 0.5)
        goal_msg.pose.pose.orientation.w = cy
        goal_msg.pose.pose.orientation.z = sy
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        
        self.get_logger().info(f'Sending goal: x={x}, y={y}, theta={theta}')
        
        # Send the goal
        self.start_time = time.time()
        self.goal_sent = True
        
        # Send goal and register callbacks
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True
    
    def _goal_response_callback(self, future):
        """Handle the goal response."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            self.goal_sent = False
            return
        
        self.get_logger().info('Goal accepted')
        
        # Wait for the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)
    
    def _get_result_callback(self, future):
        """Handle the result of the navigation."""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
            self.goal_reached = True
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
        
        self.goal_sent = False
    
    def _feedback_callback(self, feedback_msg):
        """Handle feedback from the navigation stack."""
        feedback = feedback_msg.feedback
        # Print distance remaining every 5 seconds by checking timestamp


def main():
    rclpy.init()
    
    # Parse arguments
    if len(sys.argv) < 3:
        print('Usage: ros2 run robot_navigation test_goal_navigation X Y [THETA]')
        return 1
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    
    # Create node and send goal
    test_node = RobotNavigationTest()
    
    # Wait a moment for subscribers to connect
    time.sleep(2.0)
    
    # Send the goal
    if not test_node.send_goal(x, y, theta):
        test_node.get_logger().error('Failed to send goal')
        rclpy.shutdown()
        return 1
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
