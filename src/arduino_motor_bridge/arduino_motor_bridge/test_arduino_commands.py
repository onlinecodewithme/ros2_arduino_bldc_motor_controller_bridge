#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
import asyncio
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

class ArduinoTestNode(Node):
    """
    Test node to send commands to the Arduino motor controller via ROS2.
    """
    
    def __init__(self):
        super().__init__('arduino_test')
        
        # Create callback group for services
        self.callback_group = ReentrantCallbackGroup()
        
        # Create publisher for velocity commands
        self.vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Create subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Create client for reset service
        self.reset_client = self.create_client(
            Trigger, 
            'reset_odom',
            callback_group=self.callback_group
        )
        
        # Try to check for service availability but don't block
        try:
            # Only wait for a short time
            if not self.reset_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning('Reset service not available, continuing without it')
        except Exception as e:
            self.get_logger().warning(f'Error waiting for reset service: {e}')
        
        # Create test sequence timer
        self.test_timer = self.create_timer(1.0, self.run_test_sequence)
        self.test_step = 0
        
        self.get_logger().info("Arduino Test Node initialized. Starting test sequence in 2 seconds...")
        time.sleep(2.0)  # Wait for bridge node to initialize
    
    def odom_callback(self, msg):
        """
        Process odometry data from Arduino.
        
        Args:
            msg: Odometry message
        """
        # Display odometry data
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion (simplified, assuming roll and pitch are 0)
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Simplified yaw calculation (2 * arcsin(qz) for small angles)
        yaw = 2.0 * qz if qw >= 0 else -2.0 * qz
        
        self.get_logger().info(f"Odometry: x={pos_x:.3f}, y={pos_y:.3f}, yaw={yaw:.3f}")
    
    async def reset_odometry(self):
        """
        Reset odometry to zero.
        """
        self.get_logger().info("Resetting odometry...")
        
        try:
            # Create request
            request = Trigger.Request()
            
            # Call service with timeout
            future = self.reset_client.call_async(request)
            # Set a timeout
            await asyncio.wait_for(asyncio.shield(future), 1.0)
            
            # Process result
            response = future.result()
            if response.success:
                self.get_logger().info(f"Reset successful: {response.message}")
            else:
                self.get_logger().error(f"Reset failed: {response.message}")
        except asyncio.TimeoutError:
            self.get_logger().error("Reset service timed out, continuing...")
        except Exception as e:
            self.get_logger().error(f"Reset failed with error: {e}")
    
    def send_velocity(self, linear_x, angular_z):
        """
        Send velocity command.
        
        Args:
            linear_x: Linear velocity in m/s
            angular_z: Angular velocity in rad/s
        """
        self.get_logger().info(f"Sending velocity: linear={linear_x}, angular={angular_z}")
        
        # Create Twist message
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        # Publish message
        self.vel_pub.publish(msg)
    
    def run_test_sequence(self):
        """
        Run a test sequence to exercise the Arduino.
        """
        # Test sequence steps
        if self.test_step == 0:
            # Skip reset step as it might not be available
            self.get_logger().info("STEP 0: Reset odometry (skipping)")
            # Move to next step immediately
        
        elif self.test_step == 1:
            # Move forward
            self.get_logger().info("STEP 1: Move forward slowly")
            self.send_velocity(0.1, 0.0)
        
        elif self.test_step == 3:
            # Stop
            self.get_logger().info("STEP 3: Stop")
            self.send_velocity(0.0, 0.0)
        
        elif self.test_step == 4:
            # Turn in place
            self.get_logger().info("STEP 4: Turn counterclockwise")
            self.send_velocity(0.0, 0.2)
        
        elif self.test_step == 6:
            # Stop
            self.get_logger().info("STEP 6: Stop")
            self.send_velocity(0.0, 0.0)
        
        elif self.test_step == 7:
            # Move backward
            self.get_logger().info("STEP 7: Move backward slowly")
            self.send_velocity(-0.1, 0.0)
        
        elif self.test_step == 9:
            # Stop
            self.get_logger().info("STEP 9: Stop")
            self.send_velocity(0.0, 0.0)
        
        elif self.test_step == 10:
            # Turn other direction
            self.get_logger().info("STEP 10: Turn clockwise")
            self.send_velocity(0.0, -0.2)
        
        elif self.test_step == 12:
            # Stop
            self.get_logger().info("STEP 12: Stop and end test")
            self.send_velocity(0.0, 0.0)
            # Stop the test sequence
            self.test_timer.cancel()
            self.get_logger().info("Test sequence completed")
        
        # Increment step
        self.test_step += 1

def main(args=None):
    rclpy.init(args=args)
    
    # Use a MultiThreadedExecutor to enable asynchronous callbacks
    executor = MultiThreadedExecutor()
    node = ArduinoTestNode()
    
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure to stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.vel_pub.publish(twist)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
