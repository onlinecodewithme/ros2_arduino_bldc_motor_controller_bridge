#!/usr/bin/env python3

"""
Direct script to run navigation with simulated odometry for visualization.

This script:
1. Launches map server with a simple map
2. Starts Nav2 navigation
3. Provides a simulated odometry source so virtual robot movement is visible
4. Opens RViz for visualization
5. Optionally starts Arduino bridge for real hardware control

Note: The script has been modified to use bash --login for running commands
to ensure proper environment setup when sourcing ROS2.
"""

import os
import sys
import subprocess
import signal
import time
import threading

# Path to the ROS2 workspace
WORKSPACE_PATH = os.path.join(os.path.expanduser("~"), "ros2_ws_arduino")

def run_command_in_new_terminal(command, shell=True):
    """Run a command in a new terminal and return the process"""
    # IMPORTANT: Use bash --login to ensure proper environment setup
    # Without this, the source command fails in subshells
    if shell and command.startswith("bash -c"):
        # Replace bash -c with bash --login -c to ensure proper shell environment
        command = command.replace("bash -c", "bash --login -c")
    
    process = subprocess.Popen(
        command,
        shell=shell,
        preexec_fn=os.setsid
    )
    return process

def main():
    # Make sure ROS2 is sourced
    setup_bash = os.path.join(WORKSPACE_PATH, "install", "setup.bash")
    os.environ["PATH"] = os.path.join(WORKSPACE_PATH, "install", "bin") + ":" + os.environ["PATH"]
    
    # Allow display for RViz
    os.environ["DISPLAY"] = ":1"
    
    # Start various components
    print("\n===== Starting Navigation with Simulated Map =====\n")
    processes = []
    
    # Start static transform publisher (map -> odom)
    print("Starting static transform publisher...")
    tf_cmd = f"bash -c 'source /opt/ros/humble/setup.bash && source {setup_bash} && " + \
             "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom'"
    processes.append(run_command_in_new_terminal(tf_cmd))
    time.sleep(1)
    
    # Launch map server
    print("Starting map server...")
    map_path = os.path.join(WORKSPACE_PATH, "src/robot_navigation/maps/simple_map.yaml")
    map_cmd = f"bash -c 'source /opt/ros/humble/setup.bash && source {setup_bash} && " + \
              f"ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={map_path}'"
    processes.append(run_command_in_new_terminal(map_cmd))
    time.sleep(1)
    
    # Start robot state publisher with URDF file
    print("Starting robot state publisher with robot model...")
    urdf_path = os.path.join(WORKSPACE_PATH, "src/robot_navigation/urdf/diff_robot.urdf.xacro")
    rsp_cmd = f"bash -c 'source /opt/ros/humble/setup.bash && source {setup_bash} && " + \
              f"ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"$(xacro {urdf_path})\"'"
    processes.append(run_command_in_new_terminal(rsp_cmd))
    time.sleep(2)
    
    # Launch simulated odometry publisher
    print("Starting simulated odometry publisher...")
    odom_script = os.path.join(WORKSPACE_PATH, "src/robot_navigation/robot_navigation/simulated_odometry.py")
    odom_cmd = f"bash -c 'source /opt/ros/humble/setup.bash && source {setup_bash} && " + \
               f"python3 {odom_script}'"
    processes.append(run_command_in_new_terminal(odom_cmd))
    time.sleep(1)
    
    # Arduino bridge is optional for visualization
    print("NOTE: Arduino bridge is NOT started - running in simulation only mode")
    print("      (Motor commands will be simulated but not sent to hardware)")
    
    # Launch navigation controller with bringup
    print("Starting navigation controller...")
    nav_params = os.path.join(WORKSPACE_PATH, "src/robot_navigation/config/nav2_params.yaml")
    nav_cmd = f"bash -c 'source /opt/ros/humble/setup.bash && source {setup_bash} && " + \
              f"ros2 launch nav2_bringup navigation_launch.py params_file:={nav_params} use_sim_time:=false'"
    processes.append(run_command_in_new_terminal(nav_cmd))
    time.sleep(3)
    
    # Launch RViz with navigation configuration
    print("Starting RViz for visualization...")
    
    # Try to find a navigation config file
    rviz_configs = [
        os.path.join(WORKSPACE_PATH, "src/robot_navigation/config/nav2_visualization.rviz"),
        os.path.join(WORKSPACE_PATH, "install/robot_navigation/share/robot_navigation/config/nav2_visualization.rviz"),
        "/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
    ]
    
    # Find the first config that exists
    rviz_config = next((config for config in rviz_configs if os.path.exists(config)), rviz_configs[-1])
    
    print(f"Using RViz config: {rviz_config}")
    rviz_cmd = f"bash -c 'source /opt/ros/humble/setup.bash && source {setup_bash} && " + \
               f"ros2 run rviz2 rviz2 -d {rviz_config}'"
    processes.append(run_command_in_new_terminal(rviz_cmd))
    
    print("\n===== Navigation system started =====\n")
    print("Set navigation goals in RViz using the 2D Nav Goal button")
    print("Press Ctrl+C to stop all components")
    
    try:
        # Keep the script running to maintain the processes
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down navigation system...")
        # Kill all processes
        for process in processes:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except:
                pass
        print("Navigation system stopped")

if __name__ == "__main__":
    main()
