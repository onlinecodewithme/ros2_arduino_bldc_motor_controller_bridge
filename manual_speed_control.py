#!/usr/bin/env python3

"""
ROS2 Manual Speed Control Tool
This script allows you to manually control robot speed via keyboard input.
Enter speed values for linear.x and angular.z when prompted.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ManualSpeedControl(Node):
    def __init__(self):
        super().__init__('manual_speed_control')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Manual Speed Control initialized')
        self.get_logger().info('Enter linear.x and angular.z values when prompted')
        self.get_logger().info('Use Ctrl+C to exit')
        
    def send_command(self, linear_x, angular_z):
        """Send a velocity command"""
        self.get_logger().info(f"Sending command: linear.x = {linear_x}, angular.z = {angular_z}")
        
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        # Publish command
        self.publisher.publish(msg)
        self.get_logger().info("Command sent")
        
    def stop_robot(self):
        """Stop the robot movement"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("ROBOT STOPPED")

    def run_control_loop(self):
        """Main control loop to accept user input"""
        try:
            while rclpy.ok():
                print("\n==== SPEED CONTROL MENU ====")
                print("1: Move Forward (speed 0.1)")
                print("2: Move Forward (speed 0.6)")
                print("3: Move Backward (speed -0.1)")
                print("4: Move Backward (speed -0.6)")
                print("5: Turn Left (speed 0.1)")
                print("6: Turn Right (speed -0.1)")
                print("7: Forward + Left (0.1, 0.2)")
                print("8: Forward + Right (0.1, -0.2)")
                print("9: Backward + Left (-0.1, 0.2)")
                print("0: STOP")
                print("c: Custom values")
                
                choice = input("\nEnter choice (0-9, c): ").strip()
                
                if choice == '1':
                    self.send_command(0.1, 0.0)
                elif choice == '2':
                    self.send_command(0.6, 0.0)
                elif choice == '3':
                    self.send_command(-0.1, 0.0)
                elif choice == '4':
                    self.send_command(-0.6, 0.0)
                elif choice == '5':
                    self.send_command(0.0, 0.1)
                elif choice == '6':
                    self.send_command(0.0, -0.1)
                elif choice == '7':
                    self.send_command(0.1, 0.2)
                elif choice == '8':
                    self.send_command(0.1, -0.2)
                elif choice == '9':
                    self.send_command(-0.1, 0.2)
                elif choice == '0':
                    self.stop_robot()
                elif choice.lower() == 'c':
                    try:
                        linear_x = float(input("Enter linear.x value (-1.0 to 1.0): "))
                        angular_z = float(input("Enter angular.z value (-1.0 to 1.0): "))
                        self.send_command(linear_x, angular_z)
                    except ValueError:
                        self.get_logger().error("Invalid input. Please enter numeric values.")
                else:
                    self.get_logger().warning(f"Invalid choice: {choice}")
                
        except KeyboardInterrupt:
            self.get_logger().info("Manual control terminated by user")
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    
    control = ManualSpeedControl()
    
    try:
        control.run_control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        control.stop_robot()
        control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
