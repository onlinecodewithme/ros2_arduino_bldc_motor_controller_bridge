# E38s6G5 1000B G24N Encoder Integration

This document provides instructions for using the E38s6G5 1000B G24N rotary encoders with the ROS2 Arduino Motor Bridge project.

## Overview

The E38s6G5 1000B G24N is a high-resolution quadrature encoder that provides 1000 pulses per revolution (PPR). This is a significant upgrade from basic hall effect sensors, offering much more precise position tracking and improved odometry for navigation.

## Features

- **High Resolution**: 1000 PPR (4000 counts per revolution with quadrature decoding)
- **Improved Accuracy**: ~11x more precise than the original hall effect sensors
- **True Quadrature Decoding**: Proper direction detection and fine position tracking
- **Resilient to Noise**: Better signal quality with proper pull-up resistors
- **Enhanced Navigation Performance**: More accurate odometry leads to better path planning and positioning

## Files Included

1. `e38s_encoder_improved.ino` - Arduino sketch with full E38s encoder support
2. `E38S_ENCODER_WIRING_GUIDE.md` - Detailed wiring instructions with diagrams
3. This README file with usage instructions

## How to Use

### 1. Hardware Setup

Follow the wiring instructions in the `E38S_ENCODER_WIRING_GUIDE.md` file to connect your encoders to the Arduino Mega. The guide includes:

- Pin connections for both encoders
- Visual wiring diagram
- Pull-up resistor recommendations
- Mechanical mounting tips

### 2. Upload the Code

#### Using Arduino IDE
1. Open `e38s_encoder_improved.ino` in the Arduino IDE
2. Select your Arduino Mega board and port
3. Click Upload

#### Using Arduino CLI

This system already has Arduino CLI installed at `/home/x4/bin/arduino-cli`. You can compile and upload with:

```bash
# Using arduino-cli directly (if it's in your PATH)
arduino-cli compile --fqbn arduino:avr:mega arduino_improved/e38s_encoder_improved
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega arduino_improved/e38s_encoder_improved

# Or using the full path to the executable
/home/x4/bin/arduino-cli compile --fqbn arduino:avr:mega arduino_improved/e38s_encoder_improved
/home/x4/bin/arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega arduino_improved/e38s_encoder_improved
```

If you're getting "command not found" errors when using just `arduino-cli`, you can fix this by either:
1. Starting a new terminal session
2. Manually updating your PATH in the current session:
   ```bash
   export PATH=$PATH:/home/x4/bin
   ```
3. Always using the full path as shown above

Note: The VSCode errors about "Arduino.h" are just IDE configuration issues and won't affect actual compilation in the Arduino IDE.

### 3. Testing the Encoders

After uploading:

1. Open the Serial Monitor (115200 baud)
2. Type '?' to see the menu
3. Type 'D' to view debug information
4. Manually rotate each wheel and verify the position values change
   - A full wheel rotation should result in approximately 4000 count changes
   - Direction should be correctly detected (positive/negative changes)

### 4. ROS2 Integration

The encoder sketch is fully compatible with the existing ROS2 bridge:

```bash
# Use the navigation launch file with improved odometry
./src/robot_navigation/run_nav_with_sensors.sh
```

## Tuning Parameters

You may need to adjust the following parameters in the code:

- `wheelDiameter` (line 30): Set to your wheel's actual diameter in meters
- `ticksPerRevolution` (line 29): Should be 4000 for these encoders (1000 PPR × 4)
- Pin assignments (lines 12-19): If you need to use different pins

## Troubleshooting

If you encounter issues with the encoders:

### Erratic Position Readings
- Add external 4.7kΩ pull-up resistors to each encoder channel
- Keep encoder wires away from motor power lines
- Use shielded cables for longer runs

### Incorrect Direction
- Check that encoder channels A and B are connected correctly
- If reversing direction is needed, swap the A and B connections

### No Position Changes
- Verify 5V power is reaching the encoders
- Check all wiring connections
- Ensure the encoder shaft is properly coupled to the motor

## Performance Comparison

| Feature | Original Hall Sensors | E38s6G5 Encoders |
|---------|----------------------|------------------|
| Resolution | 360 CPR | 4000 CPR |
| Position accuracy | ±1° | ±0.09° |
| Direction detection | Basic | Full quadrature |
| Noise immunity | Moderate | High with pull-ups |
| ROS2 navigation accuracy | Moderate | Excellent |

## Technical Notes

- The code uses quadrature decoding, detecting both edges of both channels
- Interrupt handlers track both the A and B channels simultaneously
- Position tracking is maintained even when the Arduino is busy with other tasks
- Calculated distance values account for the high-resolution counts
