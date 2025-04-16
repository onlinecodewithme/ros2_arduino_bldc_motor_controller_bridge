#!/usr/bin/env python3

import rclpy
import math
import time
import sys
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class NavigationTestNode(Node):
    """
    Test node for sending navigation goals and monitoring cmd_vel output.
    
    This node demonstrates:
    1. Sending a navigation goal to Nav2
    2. Monitoring the Twist commands sent to motors
    3. Tracking navigation status
    """
    
    def __init__(self):
        super().__init__('navigation_test_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Create an action client for navigation
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Subscribe to cmd_vel to monitor commands to the motors
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Variables to track navigation status
        self.is_navigating = False
        self.goal_handle = None
        self.last_cmd_vel = None
        self.cmd_vel_count = 0
        
        self.get_logger().info('Navigation Test Node initialized')
    
    def cmd_vel_callback(self, msg):
        """Callback for cmd_vel messages sent to motors"""
        self.last_cmd_vel = msg
        self.cmd_vel_count += 1
        
        # Log every 10th message to avoid flooding the console
        if self.cmd_vel_count % 10 == 0:
            self.get_logger().info(
                f'Motor commands: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}'
            )
    
    def send_navigation_goal(self, x, y, theta=0.0):
        """
        Send a navigation goal to Nav2
        
        Args:
            x, y: Goal position in meters
            theta: Goal orientation in degrees
        """
        # Wait for action server
        self.get_logger().info('Waiting for navigation action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available')
            return False
        
        # Create the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        
        # Convert angle to radians and set orientation as quaternion
        theta_rad = float(theta) * (math.pi / 180.0)
        goal_pose.pose.orientation.z = math.sin(theta_rad / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta_rad / 2.0)
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f'Sending navigation goal: x={x}, y={y}, theta={theta}°')
        
        # Reset tracking variables
        self.is_navigating = True
        self.cmd_vel_count = 0
        
        # Send the goal and register callbacks
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.is_navigating = False
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Request the goal result
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle goal result"""
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation goal reached successfully!')
        else:
            status_str = {
                1: 'ABORTED',
                2: 'CANCELED',
                3: 'REJECTED'
            }.get(status, f'UNKNOWN ({status})')
            self.get_logger().error(f'Navigation goal failed with status: {status_str}')
        
        # Log final motor command statistics
        self.get_logger().info(f'Total motor commands sent: {self.cmd_vel_count}')
        if self.last_cmd_vel:
            self.get_logger().info(
                f'Last motor command: linear.x={self.last_cmd_vel.linear.x:.3f}, '
                f'angular.z={self.last_cmd_vel.angular.z:.3f}'
            )
        
        self.is_navigating = False
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        position = feedback.current_pose.pose.position
        
        self.get_logger().info(
            f'Current position: x={position.x:.2f}, y={position.y:.2f}, '
            f'cmd_vel_count={self.cmd_vel_count}'
        )
    
    def cancel_navigation(self):
        """Cancel current navigation goal"""
        if self.is_navigating and self.goal_handle:
            self.get_logger().info('Canceling current navigation goal')
            cancel_future = self.goal_handle.cancel_goal_async()
            return True
        
        self.get_logger().warn('No active navigation to cancel')
        return False

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Parse command line arguments
    usage = """
    Usage: ros2 run robot_navigation test_goal_navigation.py X Y [THETA]
    
    X, Y: Goal coordinates in meters
    THETA: Optional goal orientation in degrees
    
    Example: ros2 run robot_navigation test_goal_navigation.py 1.0 2.0 90
    """
    
    if len(sys.argv) < 3:
        print(usage)
        rclpy.shutdown()
        return
    
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    except ValueError:
        print("Error: Coordinates must be numbers")
        print(usage)
        rclpy.shutdown()
        return
    
    # Create node and executor
    node = NavigationTestNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Start the executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Send navigation goal
        if not node.send_navigation_goal(x, y, theta):
            node.get_logger().error("Failed to send navigation goal")
            rclpy.shutdown()
            return
        
        # Wait for navigation to complete
        node.get_logger().info("Waiting for navigation to complete...")
        node.get_logger().info("Press Ctrl+C to cancel navigation")
        
        # Monitor navigation status
        while node.is_navigating:
            time.sleep(0.1)
        
        # Wait a bit before shutting down
        node.get_logger().info("Navigation complete")
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        node.get_logger().info("Navigation test interrupted")
        node.cancel_navigation()
    finally:
        # Shutdown
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()
