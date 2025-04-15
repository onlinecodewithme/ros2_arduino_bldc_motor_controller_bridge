#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import time
import sys

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 3.0  # Change command every 3 seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Motor Tester Node initialized')
        
        # Keep track of test iterations
        self.test_count = 0
        self.max_tests = 25  # Run 25 tests then exit
        
        # Define speed ranges
        self.min_speed = 0.3  # Corresponds to ~90 PWM with our scaling
        self.max_speed = 0.6  # Corresponds to ~150 PWM
        
        # Define specific test sequence for direction changes
        self.test_sequence = [
            ("FORWARD", 0.4),   # 0: Forward
            ("BACKWARD", 0.4),  # 1: Backward - tests direction change
            ("STOP", 0),        # 2: Stop
            ("FORWARD", 0.4),   # 3: Forward - tests direction change
            ("FORWARD", 0.5),   # 4: Faster forward - tests speed change 
            ("BACKWARD", 0.5),  # 5: Backward - tests direction change
            ("TURN_LEFT", 0.3), # 6: Turn left
            ("TURN_RIGHT", 0.3),# 7: Turn right - tests direction change
            ("BACKWARD", 0.4),  # 8: Backward - tests direction change
            ("FORWARD", 0.4),   # 9: Forward - tests direction change
            ("STOP", 0),        # 10: Stop
        ]
        
    def timer_callback(self):
        # Check if test sequence is complete
        if self.test_count >= len(self.test_sequence):
            self.get_logger().info('Testing complete. Shutting down...')
            self.stop_motors()
            time.sleep(1.0)
            rclpy.shutdown()
            return
            
        # Create Twist message
        twist = Twist()
        
        # Get the current test from sequence
        current_test = self.test_sequence[self.test_count]
        test_type, speed = current_test
        
        # Map test type to movement
        if test_type == "STOP":
            movement_type = 0
        elif test_type == "FORWARD":
            movement_type = 1
        elif test_type == "BACKWARD":
            movement_type = 2
        elif test_type == "TURN_LEFT":
            movement_type = 3
        elif test_type == "TURN_RIGHT":
            movement_type = 4
        else:
            movement_type = 0  # Default to stop
        
        if movement_type == 0:  # Stop
            self.get_logger().info('TEST {}: STOP'.format(self.test_count))
            # All zeros
            
        elif movement_type == 1:  # Forward
            # Use defined speed from test sequence
            self.get_logger().info('TEST {}: FORWARD at speed {:.2f}'.format(self.test_count, speed))
            twist.linear.x = speed
            
        elif movement_type == 2:  # Backward
            # Use defined speed from test sequence
            self.get_logger().info('TEST {}: BACKWARD at speed {:.2f}'.format(self.test_count, speed))
            twist.linear.x = -speed
            
        elif movement_type == 3:  # Turn left
            # Use defined speed from test sequence
            self.get_logger().info('TEST {}: TURN LEFT at speed {:.2f}'.format(self.test_count, speed))
            twist.angular.z = speed
            
        elif movement_type == 4:  # Turn right
            # Use defined speed from test sequence
            self.get_logger().info('TEST {}: TURN RIGHT at speed {:.2f}'.format(self.test_count, speed))
            twist.angular.z = -speed
            
        elif movement_type == 5:  # Combined movement
            lin_speed = random.uniform(self.min_speed/2, self.max_speed/2)
            ang_speed = random.uniform(self.min_speed/4, self.max_speed/4)
            lin_dir = 1 if random.random() > 0.5 else -1
            ang_dir = 1 if random.random() > 0.5 else -1
            
            twist.linear.x = lin_speed * lin_dir
            twist.angular.z = ang_speed * ang_dir
            
            self.get_logger().info('TEST {}: COMBINED - Linear: {:.2f}, Angular: {:.2f}'.format(
                self.test_count, twist.linear.x, twist.angular.z))
        
        # Publish the command
        self.publisher.publish(twist)
        self.test_count += 1

    def stop_motors(self):
        twist = Twist()  # All zeros by default
        self.get_logger().info('Stopping motors')
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        motor_tester = MotorTester()
        rclpy.spin(motor_tester)
    except KeyboardInterrupt:
        print('Motor test interrupted')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            if 'motor_tester' in locals():
                motor_tester.stop_motors()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
