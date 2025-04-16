#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class NavigationNode(Node):
    """
    ROS2 Node for autonomous navigation using Nav2.
    This node provides an interface to Nav2 for path planning and execution,
    and forwards velocity commands to the Arduino motor controller.
    """
    
    def __init__(self):
        super().__init__('navigation_node')
        
        # Navigation goal action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Command velocity publisher for controlling robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Setup subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create a timer to publish status
        self.create_timer(1.0, self.status_callback)
        
        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Navigation node initialized')
        self.nav_goal_handle = None
        self.is_navigating = False
    
    def status_callback(self):
        """
        Periodically check and report robot status
        """
        try:
            # Get transform from map to base_footprint to know robot position
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time())
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # Calculate heading from quaternion
            q = trans.transform.rotation
            heading = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            # Convert to degrees
            heading_deg = math.degrees(heading)
            
            self.get_logger().debug(f'Robot position: x={x:.2f}, y={y:.2f}, heading={heading_deg:.2f}°')
            
        except TransformException as ex:
            self.get_logger().debug(f'Could not get transform: {ex}')
    
    def navigate_to(self, x, y, theta=0.0):
        """
        Send a navigation goal to Nav2
        
        Args:
            x, y: Goal position coordinates
            theta: Goal orientation in radians
        """
        # Wait for navigation action server
        self.get_logger().info('Waiting for navigation action server...')
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available')
            return False
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Set orientation (convert theta to quaternion)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.get_logger().info(f'Navigating to: x={x}, y={y}, theta={theta}')
        
        # Create and send goal
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        
        self.is_navigating = True
        self.nav_goal_handle = self.nav_to_pose_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        
        # Add done callback
        self.nav_goal_handle.add_done_callback(self.goal_response_callback)
        return True
    
    def goal_response_callback(self, future):
        """Handle the goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.is_navigating = False
            return
        
        self.get_logger().info('Goal accepted')
        
        # Request the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle the goal result"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached successfully')
        else:
            self.get_logger().warning(f'Goal failed with status: {status}')
        
        self.is_navigating = False
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        
        # Log current position
        self.get_logger().debug(
            f'Current pose: '
            f'x={current_pose.position.x:.2f}, '
            f'y={current_pose.position.y:.2f}'
        )
    
    def stop_navigation(self):
        """Stop the current navigation goal"""
        if self.is_navigating and self.nav_goal_handle is not None:
            self.get_logger().info('Canceling current navigation goal')
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            
            # Send stop command to motors
            twist_msg = Twist()
            self.cmd_vel_pub.publish(twist_msg)
            
            self.is_navigating = False
            return True
        
        return False


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
