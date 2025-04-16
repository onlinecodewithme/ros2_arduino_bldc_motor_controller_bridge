#!/bin/bash
# Run navigation with simulated odometry
# This script launches navigation visualization with simulated odometry

# Kill any existing processes
pkill -f "ros2_ws_arduino.*map_server" || true
pkill -f "ros2_ws_arduino.*robot_state_publisher" || true
pkill -f "ros2_ws_arduino.*rviz" || true
pkill -f "ros2_ws_arduino.*nav2" || true
pkill -f "ros2_ws_arduino.*transform_publisher" || true
sleep 1

# Set display for RViz
export DISPLAY=:1

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws_arduino/install/setup.bash

# Workspace path
WS_PATH=~/ros2_ws_arduino

# Build the workspace
echo "Building workspace..."
cd $WS_PATH
colcon build --packages-select robot_navigation --symlink-install
source $WS_PATH/install/setup.bash

# Function to run a command in a new terminal
# NOTE: Using bash --login to ensure proper environment setup
# This is critical - without --login, the 'source' command won't work 
# in subshells and ROS2 environment won't be available
run_cmd() {
    echo "Starting: $1"
    bash --login -c "$1" &
    sleep 1
}

echo "===== Starting Navigation Simulation ====="

# Run static transform publisher
run_cmd "source /opt/ros/humble/setup.bash && source $WS_PATH/install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom"

# Run map server
MAP_PATH=$WS_PATH/src/robot_navigation/maps/simple_map.yaml
run_cmd "source /opt/ros/humble/setup.bash && source $WS_PATH/install/setup.bash && ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP_PATH"

# Run robot state publisher
URDF_PATH=$WS_PATH/src/robot_navigation/urdf/diff_robot.urdf.xacro
URDF_CMD="xacro $URDF_PATH"
URDF_CONTENT=$(eval $URDF_CMD)
run_cmd "source /opt/ros/humble/setup.bash && source $WS_PATH/install/setup.bash && ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"$URDF_CONTENT\""

# Run dummy publisher to simulate odometry
run_cmd "source /opt/ros/humble/setup.bash && source $WS_PATH/install/setup.bash && python3 -c '
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class SimpleOdomNode(Node):
    def __init__(self):
        super().__init__(\"simple_odom_node\")
        self.publisher = self.create_publisher(Odometry, \"odom\", 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        self.tf_broadcaster = TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.get_logger().info(\"Simulated odometry publisher started\")

    def timer_callback(self):
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = \"odom\"
        odom.child_frame_id = \"base_link\"
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (as quaternion)
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        
        # Publish
        self.publisher.publish(odom)
        
        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = \"odom\"
        t.child_frame_id = \"base_link\"
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

if __name__ == \"__main__\":
    main()
'"

# Start RViz
run_cmd "source /opt/ros/humble/setup.bash && source $WS_PATH/install/setup.bash && ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"

echo "===== Navigation System Running ====="
echo "Set navigation goals in RViz using the 2D Nav Goal button"
echo "Press Ctrl+C to stop all components"

# Wait for Ctrl+C
trap "echo 'Shutting down...'; pkill -f 'ros2_ws_arduino.*map_server'; pkill -f 'ros2_ws_arduino.*robot_state_publisher'; pkill -f 'ros2_ws_arduino.*rviz'; pkill -f 'ros2_ws_arduino.*nav2'; pkill -f 'ros2_ws_arduino.*transform_publisher'; exit 0" INT TERM
wait
