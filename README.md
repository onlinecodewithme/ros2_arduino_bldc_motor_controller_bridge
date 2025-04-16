# ROS2 Arduino Motor Control and Navigation

This repository contains ROS2 packages for controlling a differential drive robot using an Arduino Mega as a motor controller bridge, with integrated navigation capabilities.

## Components

### Arduino Motor Bridge

Connects ROS2 with an Arduino-based motor controller, supporting:

- **Basic Motor Control**: Command velocity to wheel velocity conversion
- **Hall Sensor Feedback**: Low-resolution odometry from hall effect sensors
- **E38S Encoder Support**: High-resolution (4000 counts/rev) quadrature encoder integration for precise positioning
- **Serial Communication**: Robust error recovery and auto-reconnection
- **Odometry Publishing**: TF and nav_msgs/Odometry for ROS2 navigation stack
- **Configurable Parameters**: Wheel size, robot dimensions, encoder resolution

### Robot Navigation

Provides navigation capabilities based on Nav2:

- **Path Planning**: Local and global planning for autonomous navigation
- **Localization**: AMCL for position tracking using maps
- **Maps**: Simple and complex map support with customizable parameters
- **Visualization**: RViz configuration for viewing navigation status 
- **Command Line Tools**: Scripts for running and testing navigation

## Latest Implementations

### 1. Robust Encoder Communication (April 2025)

Added highly resilient serial communication with advanced error recovery:

- **Port Locking**: Prevents other processes from accessing the Arduino port
- **Auto Recovery**: Automatically reconnects after communication errors
- **Fault Isolation**: Handles hardware disconnects and USB port failures
- **Watchdog Timers**: Monitors connection health and resets when needed
- **Process Management**: Detects and releases competing port access

```bash
# Run navigation with robust encoder support
ros2 launch arduino_motor_bridge robust_encoder_bridge.launch.py
```

### 2. E38S High-Resolution Encoders (April 2025)

Integrated E38S encoders for significantly improved position accuracy:

- **Increased Resolution**: 4000 ticks/revolution (11x more precise than hall sensors)
- **Improved Odometry**: Arc-based motion estimation for better tracking during turns
- **Noise Filtering**: Detects and ignores outlier measurements
- **TF Integration**: Proper frame transformations for visualization in RViz
- **Parameter Tuning**: Easy configuration of encoder parameters

```bash
# Launch navigation with E38S encoder integration
./src/robot_navigation/run_e38s_navigation.sh
```

### 3. Navigation Improvements (March 2025)

Enhanced navigation capabilities:

- **Map Support**: Added complex map handling with configurable parameters
- **Simulation Mode**: Test navigation logic without physical hardware
- **Parameter Tuning**: Optimized Nav2 parameters for better performance
- **Auto-Recovery**: Navigation automatically recovers from obstacles
- **Launch Files**: Simplified launch files for various configurations

## Setup and Usage

### Installation

1. Clone this repository to your ROS2 workspace
2. Build the packages with colcon:
```bash
cd ~/ros2_ws_arduino
colcon build --symlink-install --packages-select arduino_motor_bridge robot_navigation
```
3. Source the workspace:
```bash
source ~/ros2_ws_arduino/install/setup.bash
```

### Running the Motor Bridge

Basic bridge:
```bash
ros2 launch arduino_motor_bridge arduino_bridge.launch.py
```

With E38S encoders (robust):
```bash
ros2 launch arduino_motor_bridge robust_encoder_bridge.launch.py
```

With parameters:
```bash
ros2 launch arduino_motor_bridge robust_encoder_bridge.launch.py wheel_radius:=0.0825 base_width:=0.17 encoder_ticks_per_rev:=4000.0
```

### Running Navigation

Simple navigation:
```bash
./src/robot_navigation/run_navigation.sh
```

With encoder feedback:
```bash
./src/robot_navigation/run_e38s_navigation.sh
```

With visualization:
```bash
export DISPLAY=:1  # If needed for remote visualization
./src/robot_navigation/run_navigation_rviz.sh
```

## Troubleshooting

### Arduino Connection Issues

If you experience frequent disconnections:

1. Check USB cable and connections
2. Try a different USB port or hub
3. Ensure no other processes are using the serial port:
```bash
fuser -k /dev/ttyACM0  # Release any processes using the port
```
4. Update Arduino firmware if needed
5. Use the robust encoder implementation which has automatic reconnection capabilities

### Navigation Problems

1. Ensure odometry is working correctly (echo the /odom topic)
2. Check the transformation tree (tf) for missing transforms
3. Make sure the map is loaded correctly
4. Adjust the navigation parameters in the config files if needed
5. Verify laser scan data is being published

## Contributing

Feel free to submit issues and pull requests with improvements or bug fixes.
