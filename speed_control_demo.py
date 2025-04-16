#!/usr/bin/env python3

"""
ROS2 Speed Control Demo
This script demonstrates controlling motor speed using Twist messages.
It sends various speed commands to show how linear.x and angular.z values
affect robot movement.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpeedControlDemo(Node):
    def __init__(self):
        super().__init__('speed_control_demo')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Speed Control Demo Node initialized')
        
    def send_command(self, linear_x, angular_z, duration=3.0, description=""):
        """
        Send a velocity command and hold for specified duration
        """
        self.get_logger().info(f"COMMAND: {description}")
        self.get_logger().info(f"linear.x = {linear_x}, angular.z = {angular_z}")
        
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        
        # Publish command
        self.publisher.publish(msg)
        
        # Sleep for duration
        time.sleep(duration)
        
        # Stop briefly between commands
        self.stop_robot()
        time.sleep(1.0)
        
    def stop_robot(self):
        """Stop the robot movement"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("STOPPING")
        
    def run_demo(self):
        """
        Run through a sequence of speed demonstration commands
        """
        self.get_logger().info("Starting speed control demonstration")
        
        # Demonstration 1: Forward at different speeds
        self.send_command(0.2, 0.0, duration=3.0, description="Very slow forward")
        self.send_command(0.4, 0.0, duration=3.0, description="Slow forward")
        self.send_command(0.6, 0.0, duration=3.0, description="Medium forward")
        self.send_command(1.0, 0.0, duration=3.0, description="Maximum forward")
        
        # Demonstration 2: Backward at different speeds
        self.send_command(-0.2, 0.0, duration=3.0, description="Very slow backward")
        self.send_command(-0.5, 0.0, duration=3.0, description="Medium backward")
        self.send_command(-0.8, 0.0, duration=3.0, description="Fast backward")
        
        # Demonstration 3: Turning at different speeds
        self.send_command(0.0, 0.3, duration=3.0, description="Slow left turn in place")
        self.send_command(0.0, -0.3, duration=3.0, description="Slow right turn in place")
        
        # Demonstration 4: Combined motion
        self.send_command(0.3, 0.2, duration=3.0, description="Forward while turning left")
        self.send_command(0.3, -0.2, duration=3.0, description="Forward while turning right")
        self.send_command(-0.3, 0.2, duration=3.0, description="Backward while turning left")
        
        # Final stop
        self.stop_robot()
        self.get_logger().info("Speed control demonstration complete")

def main(args=None):
    rclpy.init(args=args)
    
    demo = SpeedControlDemo()
    
    try:
        demo.run_demo()
    except KeyboardInterrupt:
        pass
    finally:
        demo.stop_robot()
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
