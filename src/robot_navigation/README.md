# ROS2 Navigation with Arduino Motor Control

This package provides a framework for running ROS2 Navigation with an Arduino-controlled differential drive robot.

## Components

- **Arduino Motor Bridge**: Interface between ROS2 and Arduino motor controller
- **Navigation**: Map loading, path planning, and robot control
- **Simulated Odometry**: For testing navigation without hardware

## Quick Start

### Simulated Navigation (No Hardware Required)

Run the navigation simulation to visualize and test navigation goals:

```bash
./run_navigation_sim.sh
```

This will start:
- Map Server with simple map
- Robot State Publisher with differential drive robot model
- Simulated Odometry Publisher
- RViz visualization

Use RViz to set navigation goals with the 2D Nav Goal button.

### Hardware Navigation (Arduino Required)

To run with real hardware, connect Arduino and run:

```bash
# Make sure Arduino is connected
./run_navigation.sh
```

## Architecture

The system is structured as follows:

1. **Navigation Stack**
   - Map loading 
   - Path planning
   - Obstacle avoidance
   - Goal processing

2. **Robot Control**
   - Translates velocity commands (Twist) from navigation to motor commands
   - Handles Arduino communication
   - Provides feedback for odometry

3. **Odometry**
   - Either simulated (for testing) or from hall sensors (for real hardware)
   - Publishes position/orientation for navigation feedback

## Usage

### Setting Navigation Goals

1. Start the navigation system (simulated or real)
2. Open RViz (automatically launched)
3. Use the 2D Nav Goal button to set destinations
4. Watch as the robot plans and follows paths to the goal
5. Motor commands are sent to the Arduino to control the robot's movement

### Configuring Parameters

Adjust navigation parameters in:
```
config/nav2_params.yaml
```

Adjust Arduino parameters in:
```
src/arduino_motor_bridge/config/arduino_params.yaml
```

## Development

### Adding Custom Behaviors

For custom behaviors, modify:
- `robot_navigation/navigation_node.py` for navigation logic
- `arduino_motor_bridge/arduino_bridge_node.py` for hardware control

### Improving Map Support

You can replace the simple map with more complex environments:
1. Generate a map using SLAM tools
2. Place in the maps directory
3. Update the map path in launch files

## Troubleshooting

### Navigation Issues
- Ensure all transforms are correctly published
- Check map-to-odom transform is correct 
- Verify odometry data is reasonable

### Hardware Issues
- Check Arduino connection and USB port
- Verify motor wiring
- Ensure correct PID parameters

## Future Improvements

Potential improvements to add:
1. SLAM support for mapping
2. Improved localization
3. Path recording and replay
4. GUI for robot control
5. Remote monitoring via web interface
