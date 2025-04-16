# Robot Navigation with Arduino Motor Control

This package provides a complete navigation system for a robot with Arduino-controlled motors, featuring improved connectivity and error handling. It integrates ROS2 Navigation2 (Nav2) with an enhanced Arduino motor controller bridge to enable robust autonomous navigation.

## System Architecture

The system consists of the following components:

1. **Navigation Stack (Nav2)**: Provides path planning, obstacle avoidance, and localization
2. **Robot State Publisher**: Publishes the robot's TF frames for visualization and motion planning
3. **Improved Arduino Motor Bridge**: Communicates with the Arduino with better error handling and reconnection capabilities
4. **Map Server**: Loads the pre-built map for navigation
5. **Visualization**: Enhanced RViz for visualizing the robot, map, and navigation plans

## Key Improvements

The latest version includes several enhancements to improve reliability:

1. **Robust Arduino Connection**:
   - Automatic reconnection when connection is lost
   - Better error handling for serial communication
   - Exclusive port locking to prevent conflicts
   - Proper serial port cleanup on shutdown

2. **Enhanced Navigation**:
   - Fixed behavior tree XML files for navigation
   - Improved RViz visualization with more detailed views
   - Better monitoring of velocity commands to motors

3. **Diagnostic Tools**:
   - Arduino diagnostic script to isolate and troubleshoot connection issues
   - Quick test scripts to verify components individually

## Getting Started

### Prerequisites

- ROS2 Humble or newer
- Arduino connected via USB (default: `/dev/ttyACM0`)
- Arduino firmware properly configured to handle motor commands

### Running the Improved Navigation System

The easiest way to run the complete navigation system with improved Arduino connectivity is:

```bash
./src/robot_navigation/run_improved_navigation.sh
```

This script provides:
1. Better Arduino port detection and error handling
2. Reconnection capability if Arduino disconnects
3. Enhanced visualization with detailed robot state
4. Better error reporting

If your Arduino is connected to a different port:

```bash
./src/robot_navigation/run_improved_navigation.sh --serial-port=/dev/ttyACM1
```

### Running with Original Components

The original navigation system can still be run with:

```bash
./src/robot_navigation/test_with_visualization.sh
```

## Diagnostic Tools

### Arduino Connection Diagnostics

To diagnose Arduino connectivity issues separately from navigation:

```bash
./src/arduino_motor_bridge/arduino_diagnostic.sh
```

Options:
- `--port=/dev/ttyXXX` - Specify Arduino port
- `--verbose` - Show detailed debug information
- `--reset-arduino` - Reset Arduino before connecting

This tool will:
- Check port permission issues
- Detect if the port is in use by other processes
- Test and verify communication with Arduino
- Provide detailed error information

## Setting Navigation Goals

### Using RViz

When the navigation system is running with RViz, you can:

1. Click on the "2D Nav Goal" button in the RViz toolbar
2. Click and drag on the map to set a goal position and orientation
3. The robot will navigate to the goal, sending velocity commands to the Arduino

### Using Command Line

For command line goal setting, you can use the navigation CLI:

```bash
# In a new terminal
source install/setup.bash
ros2 run robot_navigation navigation_cli
```

Then use these commands:
- `goto X Y [THETA]` - Navigate to position X,Y with optional orientation THETA in degrees
- `cancel` - Cancel the current navigation goal
- `help` - Display available commands

## Improved Arduino Motor Bridge

The improved Arduino Motor Bridge now features:

1. **Robust Serial Communication**:
   - Automatic reconnection if connection is lost
   - Better error handling with appropriate feedback
   - Port locking to prevent conflicts with other processes

2. **Enhanced Data Processing**:
   - More reliable parsing of Arduino responses
   - Better handling of malformed data
   - Improved thread safety with proper locking

3. **Safety Features**:
   - Automatic motor stop if no commands received recently
   - Better shutdown sequence to ensure motors are stopped
   - Protection against communication errors affecting robot behavior

## Troubleshooting

### Display/GUI Issues

If you encounter issues with RViz or other GUI components:
- Ensure the DISPLAY environment variable is set correctly (e.g., `export DISPLAY=:1`)
- All scripts now automatically set this, but if you run individual components manually, you may need to set it yourself

### Arduino Connection Issues

If the system can't connect to your Arduino:
- Run the Arduino diagnostic script to identify the issue: `./src/arduino_motor_bridge/arduino_diagnostic.sh`
- Check if the Arduino is properly connected with `ls -l /dev/ttyACM*`
- Try a different port using `--serial-port=/dev/ttyXXX` argument
- Ensure you have appropriate permissions (you may need to add your user to the `dialout` group)
- Reset the Arduino by unplugging and plugging it back in
- Check if other processes are using the port with `lsof /dev/ttyACM0`

### Navigation Issues

- Check for errors in the behavior tree initialization
- Make sure the map and robot are properly configured
- Verify TF frames are published correctly (look at the TF tree in RViz)
- Look at the navigation costmaps for obstacles or planning issues
- Watch for errors in the terminal output

## Command Reference

| Command | Description |
|---------|-------------|
| `./src/robot_navigation/run_improved_navigation.sh` | Run improved navigation system with enhanced Arduino bridge |
| `./src/robot_navigation/test_with_visualization.sh` | Run original navigation system with RViz |
| `./src/arduino_motor_bridge/arduino_diagnostic.sh` | Test and diagnose Arduino connectivity issues |
| `ros2 run arduino_motor_bridge improved_arduino_bridge` | Run improved Arduino bridge node directly |
| `ros2 run robot_navigation navigation_cli` | Command line interface for navigation |
| `ros2 run robot_navigation test_goal_navigation X Y THETA` | Test navigation with specific goal |

## Advanced Configuration

### Behavior Trees

The navigation system uses custom behavior trees located in `config/`:
- `navigate_w_replanning.xml` - Used for single pose navigation
- `navigate_through_poses.xml` - Used for waypoint following

### RViz Configuration

The enhanced RViz configuration in `config/nav2_visualization.rviz` provides:
- Visualization of the map and costmaps
- Robot model and TF frames
- Path planning visualization
- Velocity command monitoring
- Localization information (particle clouds)
- Motor command visualization

### Customizing Arduino Integration

The Arduino bridge can be customized in:
- Original implementation:
  - `src/arduino_motor_bridge/arduino_motor_bridge/arduino_bridge_node.py`
  - `src/arduino_motor_bridge/arduino_motor_bridge/arduino_serial.py`
- Improved implementation:
  - `src/arduino_motor_bridge/arduino_motor_bridge/improved_bridge_node.py`
  - `src/arduino_motor_bridge/arduino_motor_bridge/improved_arduino_serial.py`

Parameters can be adjusted in:
- `src/arduino_motor_bridge/config/arduino_params.yaml`
