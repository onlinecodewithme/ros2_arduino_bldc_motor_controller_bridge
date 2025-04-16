#!/usr/bin/env python3

"""
Direct ROS2 Navigation Simulation Launcher

This script directly launches all components needed for a simulated navigation:
- Static transform publisher (map -> odom)
- Map server
- Robot state publisher
- Simple simulated odometry publisher
- RViz2 for visualization

Usage:
  python3 direct_simulation.py
"""

import os
import sys
import math
import threading
import time
import signal
import subprocess
from threading import Thread

# Path to ROS workspace
ROS_WS = os.path.expanduser("~/ros2_ws_arduino")
DISPLAY = ":1"  # X11 display for visualization

def run_command(cmd, shell=True):
    """Run a command in the background and return the process"""
    print(f"Running: {cmd}")
    return subprocess.Popen(cmd, shell=shell)

def simple_odometry():
    """Run a simple simulated odometry publisher"""
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    import numpy as np
    from math import sin, cos

    class SimpleOdomNode(Node):
        def __init__(self):
            super().__init__('simple_odom_node')
            self.publisher = self.create_publisher(Odometry, 'odom', 10)
            self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
            self.tf_broadcaster = TransformBroadcaster(self)
            
            # Robot state
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.get_logger().info('Simulated odometry publisher started')

        def timer_callback(self):
            # Simulate small random movements
            timestamp = self.get_clock().now().to_msg()
            
            # Update position with small random changes
            self.x += np.random.normal(0, 0.0001)  # Small random changes
            self.y += np.random.normal(0, 0.0001)
            self.theta += np.random.normal(0, 0.0001)
            
            # Publish odometry message
            odom = Odometry()
            odom.header.stamp = timestamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            # Position
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            
            # Orientation (as quaternion)
            cy = cos(self.theta * 0.5)
            sy = sin(self.theta * 0.5)
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = sy
            odom.pose.pose.orientation.w = cy
            
            # Publish
            self.publisher.publish(odom)
            
            # Broadcast transform
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)
            
    def main():
        rclpy.init()
        node = SimpleOdomNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
            
    main()

def main():
    """Launch all components of the navigation stack"""
    
    # Store all processes to terminate them properly
    processes = []
    
    # Set display for visualization
    os.environ["DISPLAY"] = DISPLAY
    
    # Source ROS2 workspace
    source_cmd = f"source /opt/ros/humble/setup.bash && source {ROS_WS}/install/setup.bash"
    
    try:
        print("\n==== Starting Navigation Simulation ====\n")
        
        # Start static transform publisher (map -> odom)
        cmd = f"{source_cmd} && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom"
        processes.append(run_command(cmd))
        time.sleep(1)
        
        # Start map server
        map_path = f"{ROS_WS}/src/robot_navigation/maps/simple_map.yaml"
        cmd = f"{source_cmd} && ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={map_path}"
        processes.append(run_command(cmd))
        time.sleep(1)
        
        # Start robot state publisher
        urdf_path = f"{ROS_WS}/src/robot_navigation/urdf/diff_robot.urdf.xacro"
        cmd = f"{source_cmd} && ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"$(xacro {urdf_path})\""
        processes.append(run_command(cmd))
        time.sleep(2)
        
        # Start simulated odometry publisher in a separate thread
        print("Starting simulated odometry in a thread...")
        odometry_thread = Thread(target=simple_odometry)
        odometry_thread.daemon = True
        odometry_thread.start()
        time.sleep(2)
        
        # Start RViz
        cmd = f"{source_cmd} && ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
        processes.append(run_command(cmd))
        
        print("\n==== Navigation System Running ====")
        print("Set navigation goals in RViz using the 2D Nav Goal button")
        print("Press Ctrl+C to stop all components\n")
        
        # Keep running until interrupted
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down navigation system...")
    finally:
        # Terminate all processes
        for process in processes:
            try:
                process.terminate()
                process.wait(timeout=2)
            except:
                process.kill()
        print("Navigation system stopped")

if __name__ == "__main__":
    main()
