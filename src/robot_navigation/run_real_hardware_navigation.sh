#!/bin/bash
# Run navigation with real hardware
# This script launches navigation with Arduino hardware connection for motor control

# Kill any existing processes
pkill -f "ros2_ws_arduino.*map_server" || true
pkill -f "ros2_ws_arduino.*robot_state_publisher" || true
pkill -f "ros2_ws_arduino.*rviz" || true
pkill -f "ros2_ws_arduino.*nav2" || true
pkill -f "ros2_ws_arduino.*transform_publisher" || true
pkill -f "ros2_ws_arduino.*arduino_bridge" || true
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
colcon build --packages-select robot_navigation arduino_motor_bridge --symlink-install
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

echo "===== Starting Navigation with Real Hardware =====\n"

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

# Run Arduino bridge
echo "Starting Arduino bridge for real hardware control..."
# Try to find the Arduino port
ARDUINO_PORT="/dev/ttyACM0" # Default port
for port in /dev/ttyACM*; do
    if [ -e "$port" ]; then
        ARDUINO_PORT="$port"
        break
    fi
done
echo "Using Arduino port: $ARDUINO_PORT"

run_cmd "source /opt/ros/humble/setup.bash && source $WS_PATH/install/setup.bash && ros2 run arduino_motor_bridge arduino_bridge_node --ros-args -p serial_port:=$ARDUINO_PORT"

# Launch navigation controller
echo "Starting navigation controller..."
nav_params=$WS_PATH/src/robot_navigation/config/nav2_params.yaml
run_cmd "source /opt/ros/humble/setup.bash && source $WS_PATH/install/setup.bash && ros2 launch nav2_bringup navigation_launch.py params_file:=$nav_params use_sim_time:=false"
sleep 2

# Start RViz
echo "Starting RViz for visualization..."
rviz_configs=(
    "$WS_PATH/src/robot_navigation/config/nav2_visualization.rviz"
    "$WS_PATH/install/robot_navigation/share/robot_navigation/config/nav2_visualization.rviz"
    "/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
)

# Find the first config that exists
rviz_config=""
for config in "${rviz_configs[@]}"; do
    if [ -f "$config" ]; then
        rviz_config="$config"
        break
    fi
done

if [ -z "$rviz_config" ]; then
    rviz_config="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
fi

echo "Using RViz config: $rviz_config"
run_cmd "source /opt/ros/humble/setup.bash && source $WS_PATH/install/setup.bash && ros2 run rviz2 rviz2 -d $rviz_config"

echo "===== Real Hardware Navigation System Running ====="
echo "Set navigation goals in RViz using the 2D Nav Goal button"
echo "The navigation stack will send velocity commands to Arduino motor controller"
echo "Press Ctrl+C to stop all components"

# Wait for Ctrl+C
trap "echo 'Shutting down...'; pkill -f 'ros2_ws_arduino.*map_server'; pkill -f 'ros2_ws_arduino.*robot_state_publisher'; pkill -f 'ros2_ws_arduino.*rviz'; pkill -f 'ros2_ws_arduino.*nav2'; pkill -f 'ros2_ws_arduino.*transform_publisher'; pkill -f 'ros2_ws_arduino.*arduino_bridge'; exit 0" INT TERM
wait
