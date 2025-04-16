#!/usr/bin/env python3

import math
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import threading


class NavigationCLI(Node):
    """
    Command Line Interface for interacting with the navigation stack.
    This node provides a simple CLI for sending navigation goals to Nav2.
    """
    
    def __init__(self):
        super().__init__('navigation_cli')
        
        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Initialize navigation status
        self.is_navigating = False
        self.goal_handle = None
        
        # Flag to signal program exit
        self.should_exit = False
        
        self.get_logger().info('Navigation CLI initialized')
        self.get_logger().info('Enter "help" for available commands')
    
    def send_goal(self, x, y, theta=0.0):
        """Send a navigation goal to Nav2"""
        # Wait for the action server to be available
        self.get_logger().info('Waiting for navigation action server...')
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False
        
        # Create the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set the target position
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Set the target orientation (as a quaternion)
        theta_rad = float(theta) * (math.pi / 180.0)  # Convert to radians if given in degrees
        goal_msg.pose.pose.orientation.z = math.sin(theta_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta_rad / 2.0)
        
        self.get_logger().info(f'Sending goal: x={x}, y={y}, theta={theta}')
        
        # Send the goal
        self.is_navigating = True
        goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        goal_future.add_done_callback(self.goal_response_callback)
        return True
    
    def goal_response_callback(self, future):
        """Process the goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            self.is_navigating = False
            return
        
        self.get_logger().info('Goal accepted!')
        
        # Request the result
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle the goal result"""
        status = future.result().status
        
        if status == 4:  # 4 = SUCCEEDED
            self.get_logger().info('Goal completed successfully!')
        else:
            status_str = {
                1: 'ABORTED',
                2: 'CANCELED',
                3: 'REJECTED'
            }.get(status, f'UNKNOWN ({status})')
            self.get_logger().error(f'Goal failed with status: {status_str}')
        
        self.is_navigating = False
    
    def feedback_callback(self, feedback_msg):
        """Process navigation feedback"""
        feedback = feedback_msg.feedback
        position = feedback.current_pose.pose.position
        
        # Report current position (only occasionally to avoid flooding the console)
        self.get_logger().info(f'Current position: x={position.x:.2f}, y={position.y:.2f}')
    
    def cancel_navigation(self):
        """Cancel the current navigation goal"""
        if self.is_navigating and self.goal_handle:
            self.get_logger().info('Cancelling current goal')
            
            # Send cancel request
            self.goal_handle.cancel_goal_async()
            return True
        else:
            self.get_logger().warn('No active navigation to cancel')
            return False
    
    def print_help(self):
        """Display available commands"""
        help_text = """
        Navigation CLI Commands:
        ---------------------
        goto X Y [THETA]   - Navigate to position X, Y with optional orientation THETA in degrees
        cancel             - Cancel the current navigation goal
        help               - Display this help message
        exit               - Exit the program
        
        Examples:
        --------
        goto 1.0 2.0       - Navigate to position x=1.0, y=2.0
        goto -3.0 5.0 90   - Navigate to position x=-3.0, y=5.0 with 90° orientation
        """
        print(help_text)
    
    def cli_loop(self):
        """Main CLI input loop"""
        while not self.should_exit:
            try:
                # Get user input
                user_input = input("\nEnter command > ").strip()
                parts = user_input.split()
                
                if not parts:
                    continue
                
                command = parts[0].lower()
                
                if command == 'goto' and len(parts) >= 3:
                    # Parse coordinates
                    try:
                        x = float(parts[1])
                        y = float(parts[2])
                        theta = float(parts[3]) if len(parts) > 3 else 0.0
                        self.send_goal(x, y, theta)
                    except ValueError:
                        self.get_logger().error("Invalid coordinates. Use: goto X Y [THETA]")
                
                elif command == 'cancel':
                    self.cancel_navigation()
                
                elif command in ['help', '?']:
                    self.print_help()
                
                elif command in ['exit', 'quit']:
                    self.should_exit = True
                    print("Exiting...")
                    break
                
                else:
                    self.get_logger().error("Unknown command. Type 'help' for available commands.")
            
            except KeyboardInterrupt:
                print("\nKeyboard interrupt detected.")
                self.should_exit = True
                break
            except Exception as e:
                self.get_logger().error(f"Error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    cli_node = NavigationCLI()
    
    # Set up threading
    executor = MultiThreadedExecutor()
    executor.add_node(cli_node)
    
    # Start the ROS2 executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Run the CLI in the main thread
        cli_node.cli_loop()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
