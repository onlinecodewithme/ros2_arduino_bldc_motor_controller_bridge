# ROS2 Arduino Motor Bridge with Navigation

A robust bridge between ROS2 and Arduino-controlled motors with enhanced motor control capabilities and full navigation stack integration.

## System Overview

This system provides a reliable interface between ROS2 and Arduino-controlled DC motors, with added navigation capabilities. It features:

- Bidirectional communication using serial protocol
- Velocity commands using ROS2 standard Twist messages
- Odometry feedback from motor encoders
- Smooth direction transitions with appropriate speed ramping
- Specialized handling for backward motion
- Stable motor control with proper minimum speeds
- Watchdog safety feature to stop motors on communication loss
- **Full ROS2 Navigation2 (Nav2) integration for autonomous navigation**
- **Improved Arduino connectivity with auto-reconnection**
- **Navigation visualization tools with RViz**

## Hardware Requirements

- Arduino Uno/Mega or compatible board
- Motor driver capable of controlling two DC motors
- DC motors with hall effect encoders
- 12V Power supply (or appropriate voltage for your motors)
- USB cable for Arduino-computer connection

### Pin Connections

The Arduino code uses the following pins by default:

- **PWM Output Pins**:
  - Left Motor: Pin 9
  - Right Motor: Pin 10

- **Direction Control Pins**:
  - Left Motor: Pin 7
  - Right Motor: Pin 8

- **Hall Sensor Pins**:
  - Left Motor: Pin 2
  - Right Motor: Pin 5

> **Note for Arduino Mega users**: These pin assignments are compatible with both Arduino Uno and Mega. If you need to use different pins on your Mega, modify the pin definitions in the Arduino sketch.

## Software Setup

### Prerequisites

- ROS2 Humble or newer
- Python 3.8+
- `pyserial` package
- Arduino IDE or arduino-cli for firmware upload
- **Navigation2 (Nav2) stack installed**

### Installation

1. **Clone the repository into your ROS2 workspace**:
   ```bash
   cd ~/ros2_ws/src/
   git clone https://github.com/yourusername/ros2_arduino_motor_bridge.git
   ```

2. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select arduino_motor_bridge robot_navigation
   ```

3. **Check your connected Arduino board**:
   ```bash
   arduino-cli board list
   ```
   This will show which board you have connected and the port it's using.

4. **Upload the Arduino firmware**:
   - Using Arduino IDE:
     - Open `arduino_improved/arduino_improved.ino`
     - Select your Arduino board and port
     - Click Upload

   - Using arduino-cli:
     - For Arduino Uno:
       ```bash
       arduino-cli compile --fqbn arduino:avr:uno arduino_improved
       arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino_improved
       ```
     
     - For Arduino Mega:
       ```bash
       arduino-cli compile --fqbn arduino:avr:mega arduino_improved
       arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega arduino_improved
       ```

5. **Source your workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Basic Motor Control Usage

### Starting the Bridge

Run the ROS2 bridge node:

```bash
ros2 run arduino_motor_bridge arduino_bridge_node
```

Or use the launch file (recommended):

```bash
ros2 launch arduino_motor_bridge arduino_bridge.launch.py
```

For the improved version with better error handling and reconnection capabilities:

```bash
ros2 launch arduino_motor_bridge improved_arduino_bridge.launch.py
```

### Controlling the Motors

The bridge subscribes to the standard `/cmd_vel` topic and accepts `geometry_msgs/Twist` messages:

- **Linear.x**: Forward/backward motion (-1.0 to 1.0)
- **Angular.z**: Rotation left/right (-1.0 to 1.0)

### Example Commands

1. **Move forward**:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   ```

2. **Move backward**:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   ```

3. **Turn left**:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'
   ```

4. **Turn right**:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.3}}'
   ```

5. **Combined motion (forward while turning)**:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'
   ```

6. **Stop**:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   ```

## Navigation System

The project now includes a complete navigation system that integrates with the Arduino motor bridge, allowing for autonomous navigation with path planning, obstacle avoidance, and localization.

### Running Navigation with Arduino Bridge

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

### Alternative Navigation Launch Options

For different navigation configurations:

1. **Basic Navigation with Visualization**:
   ```bash
   ./src/robot_navigation/test_with_visualization.sh
   ```

2. **Run Navigation with Hall Sensor Odometry**:
   ```bash
   ./src/robot_navigation/run_nav_with_sensors.sh
   ```

3. **Simulation Navigation (without physical hardware)**:
   ```bash
   ./src/robot_navigation/run_simulation_navigation.sh
   ```

4. **Real Hardware Navigation with All Components**:
   ```bash
   ./src/robot_navigation/run_real_hardware_navigation.sh
   ```

### Setting Navigation Goals

#### Using RViz

When the navigation system is running with RViz, you can:

1. Click on the "2D Nav Goal" button in the RViz toolbar
2. Click and drag on the map to set a goal position and orientation
3. The robot will navigate to the goal, sending velocity commands to the Arduino

#### Using Command Line

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

### Navigation System Components

The navigation system consists of:

1. **Navigation Stack (Nav2)**: Provides path planning, obstacle avoidance, and localization
2. **Robot State Publisher**: Publishes the robot's TF frames for visualization and motion planning
3. **Improved Arduino Motor Bridge**: Communicates with the Arduino with better error handling and reconnection capabilities
4. **Map Server**: Loads the pre-built map for navigation
5. **Visualization**: Enhanced RViz for visualizing the robot, map, and navigation plans

## Improved Arduino Bridge for Navigation

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

## Diagnostic and Troubleshooting Tools

### Arduino Connection Diagnostic

To diagnose Arduino connectivity issues separately from navigation:

```bash
./src/arduino_motor_bridge/arduino_diagnostic.sh
```

Options:
- `--port=/dev/ttyXXX` - Specify Arduino port
- `--verbose` - Show detailed debug information
- `--reset-arduino` - Reset Arduino before connecting

### Debug Motor Control

For testing motor control in detail:

```bash
./src/arduino_motor_bridge/debug_motor_control.sh
```

### Test Hall Sensor

To test hall sensor odometry function:

```bash
python3 src/arduino_motor_bridge/test_hall_sensor.py
```

### Automated Motor Testing

The package includes a test script to verify motor functionality:

```bash
python3 test_motor_control.py
```

## Troubleshooting

### Motors Don't Move

- Check Arduino power supply
- Verify motor driver connections
- Ensure minimum speeds are high enough (at least 70-90 PWM)
- Check serial connection with `ls -l /dev/ttyACM*`

### Direction Changes Don't Work

- Try increasing delay times in the direction change code
- Increase the initial pulse for backward movement
- Check direction pin connections

### Serial Communication Issues

- Verify correct port with `ls -l /dev/ttyACM*`
- Try resetting the Arduino
- Check USB cable connection
- Ensure baud rates match (115200 by default)

### Arduino Upload Issues

- Make sure you're using the correct board identifier:
  - Arduino Uno: `arduino:avr:uno`
  - Arduino Mega: `arduino:avr:mega`
- Check which board you have with `arduino-cli board list`
- Verify the port with `ls -l /dev/ttyACM*`

### Navigation Issues

- Check for errors in the behavior tree initialization
- Make sure the map and robot are properly configured
- Verify TF frames are published correctly (look at the TF tree in RViz)
- Look at the navigation costmaps for obstacles or planning issues
- Watch for errors in the terminal output

### Display/GUI Issues

If you encounter issues with RViz or other GUI components:
- Ensure the DISPLAY environment variable is set correctly (e.g., `export DISPLAY=:1`)
- All scripts now automatically set this, but if you run individual components manually, you may need to set it yourself

## Parameter Configuration

### Arduino Bridge Parameters

The bridge node can be configured using ROS2 parameters:

- **serial_port**: Arduino serial port (default: `/dev/ttyACM0`)
- **baud_rate**: Serial communication speed (default: `115200`)
- **cmd_vel_topic**: Topic name for velocity commands (default: `/cmd_vel`)
- **odom_topic**: Topic name for odometry (default: `/odom`)
- **frame_id**: TF frame for odometry (default: `odom`)
- **child_frame_id**: TF frame for robot (default: `base_link`)

You can set these parameters in the launch file or via the command line.

### Navigation Parameters

Navigation parameters can be customized in:
- `src/robot_navigation/config/nav2_params.yaml`

### Advanced Configuration

#### Behavior Trees

The navigation system uses custom behavior trees located in `config/`:
- `navigate_w_replanning.xml` - Used for single pose navigation
- `navigate_through_poses.xml` - Used for waypoint following

#### RViz Configuration

The enhanced RViz configuration in `config/nav2_visualization.rviz` provides:
- Visualization of the map and costmaps
- Robot model and TF frames
- Path planning visualization
- Velocity command monitoring
- Localization information (particle clouds)
- Motor command visualization

## Tuning Motor Parameters

For optimal performance, you may need to adjust the following parameters in the Arduino code:

- **minMotorSpeed**: Minimum PWM for forward movement (default: 90)
- **minBackwardSpeed**: Minimum PWM for backward movement (default: 110)
- **baseSpeed**: Default speed for manual control (default: 150)

If your motors don't rotate reliably in certain directions:
1. Increase the minimum speed values
2. Check motor driver connections
3. Ensure power supply is adequate

## Implemented Improvements

The current version includes several key improvements:

1. **Enhanced Direction Changes**:
   - Added complete stop before direction change
   - Implemented graduated acceleration after direction changes
   - Added higher initial pulses for backward movement

2. **Optimized Speed Settings**:
   - Separate minimum speeds for forward (90) and backward (110) movement
   - Dynamic speed adjustment based on direction

3. **Improved Reliability**:
   - Watchdog timer to stop motors on communication loss
   - Proper error handling for serial communication
   - Enhanced odometry data format

4. **Navigation Integration**:
   - Full integration with Nav2 stack
   - Improved visualization
   - Simulated and real hardware options
   - Command-line interface for goal setting

## Arduino Board Advantages

### Arduino Uno
- Smaller form factor
- Lower power consumption
- Sufficient for basic motor control applications

### Arduino Mega
- More memory (253,952 bytes vs 32,256 bytes program space)
- More I/O pins for additional sensors or motors
- Better for complex applications with multiple peripherals
- Current sketch uses only 3% of program storage on Mega vs 23% on Uno

## License

This project is licensed under the MIT License - see the LICENSE file for details.
