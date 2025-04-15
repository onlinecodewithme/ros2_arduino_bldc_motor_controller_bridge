# ROS2 Arduino Motor Bridge

A robust bridge between ROS2 and Arduino-controlled motors with enhanced motor control capabilities.

## System Overview

This system provides a reliable interface between ROS2 and Arduino-controlled DC motors. It features:

- Bidirectional communication using serial protocol
- Velocity commands using ROS2 standard Twist messages
- Odometry feedback from motor encoders
- Smooth direction transitions with appropriate speed ramping
- Specialized handling for backward motion
- Stable motor control with proper minimum speeds
- Watchdog safety feature to stop motors on communication loss

## Hardware Requirements

- Arduino Uno or compatible board
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

## Software Setup

### Prerequisites

- ROS2 Humble or newer
- Python 3.8+
- `pyserial` package
- Arduino IDE or arduino-cli for firmware upload

### Installation

1. **Clone the repository into your ROS2 workspace**:
   ```bash
   cd ~/ros2_ws/src/
   git clone https://github.com/yourusername/ros2_arduino_motor_bridge.git
   ```

2. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select arduino_motor_bridge
   ```

3. **Upload the Arduino firmware**:
   - Using Arduino IDE:
     - Open `arduino_improved/arduino_improved.ino`
     - Select your Arduino board and port
     - Click Upload

   - Using arduino-cli:
     ```bash
     arduino-cli compile --fqbn arduino:avr:uno arduino_improved
     arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino_improved
     ```

4. **Source your workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Starting the Bridge

Run the ROS2 bridge node:

```bash
ros2 run arduino_motor_bridge arduino_bridge_node
```

Or use the launch file (recommended):

```bash
ros2 launch arduino_motor_bridge arduino_bridge.launch.py
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

## Automated Testing

The package includes a test script to verify motor functionality:

```bash
python3 test_motor_control.py
```

This will run a sequence of movement tests including:
- Forward motion
- Backward motion
- Direction changes
- Turning
- Stop commands

## Parameter Configuration

The bridge node can be configured using ROS2 parameters:

- **serial_port**: Arduino serial port (default: `/dev/ttyACM0`)
- **baud_rate**: Serial communication speed (default: `115200`)
- **cmd_vel_topic**: Topic name for velocity commands (default: `/cmd_vel`)
- **odom_topic**: Topic name for odometry (default: `/odom`)
- **frame_id**: TF frame for odometry (default: `odom`)
- **child_frame_id**: TF frame for robot (default: `base_link`)

You can set these parameters in the launch file or via the command line.

## Tuning Motor Parameters

For optimal performance, you may need to adjust the following parameters in the Arduino code:

- **minMotorSpeed**: Minimum PWM for forward movement (default: 90)
- **minBackwardSpeed**: Minimum PWM for backward movement (default: 110)
- **baseSpeed**: Default speed for manual control (default: 150)

If your motors don't rotate reliably in certain directions:
1. Increase the minimum speed values
2. Check motor driver connections
3. Ensure power supply is adequate

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

### Encoder Feedback Issues

- Verify hall sensor connections
- Check interrupt pin assignment
- Test encoders independently with a simple test sketch

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

## License

This project is licensed under the MIT License - see the LICENSE file for details.
