# E38s6G5 1000B G24N Rotary Encoder Wiring Guide

This guide provides detailed instructions on how to wire the E38s6G5 1000B G24N rotary encoders to an Arduino for the ROS2 motor controller project.

## Encoder Specifications

The E38s6G5 1000B G24N is a high-resolution incremental rotary encoder with the following specifications:

- **Resolution**: 1000 Pulses Per Revolution (PPR)
- **Output**: Quadrature output (two channels, A and B)
- **Power supply**: 5-24V DC
- **Output type**: Usually NPN open-collector or Line Driver
- **Mechanical mounting**: Typically 6mm shaft diameter with mounting holes

## Wiring Diagram

Each encoder requires the following connections:

### Power Connections
- **Red wire**: +5V from Arduino
- **Black wire**: GND (ground) from Arduino

### Signal Connections
- **Green wire**: Channel A - Connect to interrupt-capable pin
- **White wire**: Channel B - Connect to interrupt-capable pin
- **Shield/Bare wire** (if present): Connect to GND for noise protection

## Connections to Arduino Mega

For Arduino Mega, we'll use the following pins:

### Left Motor Encoder
1. **Red wire** → 5V on Arduino
2. **Black wire** → GND on Arduino
3. **Green wire** (Channel A) → Digital Pin 2 (interrupt capable)
4. **White wire** (Channel B) → Digital Pin 3 (interrupt capable)

### Right Motor Encoder
1. **Red wire** → 5V on Arduino 
2. **Black wire** → GND on Arduino
3. **Green wire** (Channel A) → Digital Pin 18 (interrupt capable)
4. **White wire** (Channel B) → Digital Pin 19 (interrupt capable)

## Visual Wiring Diagram

```
LEFT ENCODER                 ARDUINO MEGA
┌────────────┐               ┌──────────────────┐
│            │               │                  │
│ RED       ┌┼──────────────►│ 5V               │
│ BLACK     ┌┼──────────────►│ GND              │
│ GREEN(A)  ┌┼──────────────►│ PIN 2 (INT0)     │
│ WHITE(B)  ┌┼──────────────►│ PIN 3 (INT1)     │
│            │               │                  │
└────────────┘               │                  │
                             │                  │
RIGHT ENCODER                │                  │
┌────────────┐               │                  │
│            │               │                  │
│ RED       ┌┼──────────────►│ 5V               │
│ BLACK     ┌┼──────────────►│ GND              │
│ GREEN(A)  ┌┼──────────────►│ PIN 18 (INT5)    │
│ WHITE(B)  ┌┼──────────────►│ PIN 19 (INT4)    │
│            │               │                  │
└────────────┘               └──────────────────┘
```

## Pull-up Resistors

The code already enables internal pull-up resistors on the encoder pins. However, for better noise immunity in longer wire runs, you may want to add external pull-up resistors:

- Connect a 4.7kΩ resistor between each signal wire (green and white) and 5V

## Mechanical Installation

1. Mount the encoders securely to your motor shafts using appropriate couplings
2. Ensure that the encoder shaft is properly aligned with the motor shaft to avoid mechanical stress
3. Use the mounting holes on the encoder to secure it to a fixed bracket

## Verifying Connections

After wiring and uploading the code:

1. Open the Arduino Serial Monitor at 115200 baud
2. Enter '?' to see the menu
3. Enter 'D' to see debug information
4. Manually rotate each motor shaft and verify that the position counts change
   - Clockwise rotation should increase the position count
   - Counter-clockwise rotation should decrease the position count

## Troubleshooting

### No Position Change Detected
- Check all wiring connections
- Verify the encoder is receiving 5V power (measure between red and black wires)
- Ensure the encoder shaft is rotating
- Try swapping A and B channels if counts go in the wrong direction

### Erratic Position Changes
- Add external pull-up resistors (4.7kΩ) to the signal lines
- Check for loose connections
- Use shielded cable for longer wire runs
- Keep encoder wires away from motor power wires

### Arduino Pin Selection Notes

If you need to use different pins, remember:
- Pins must support interrupts (on Arduino Mega: 2, 3, 18, 19, 20, 21)
- Update the pin definitions in the code:
  ```c
  const int leftEncoderPinA = 2;   // Change to your new pin
  const int leftEncoderPinB = 3;   // Change to your new pin
  const int rightEncoderPinA = 18; // Change to your new pin
  const int rightEncoderPinB = 19; // Change to your new pin
  ```

## Code Changes

The code has been updated to:
1. Use quadrature decoding for higher accuracy
2. Account for the higher resolution (1000 PPR → 4000 counts per revolution)
3. Calculate distance using the more precise encoder measurements

## Uploading the Code

### Arduino IDE Method
- Open `e38s_encoder_improved.ino` in the Arduino IDE
- Select your Arduino board (Arduino Mega recommended)
- Select the correct port
- Click the Upload button

### Arduino CLI Method

This system already has Arduino CLI installed at `/home/x4/bin/arduino-cli`. To compile and upload:

```bash
# Using arduino-cli directly (if it's in your PATH)
arduino-cli compile --fqbn arduino:avr:mega arduino_improved/e38s_encoder_improved
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega arduino_improved/e38s_encoder_improved

# Or using the full path to the executable (most reliable)
/home/x4/bin/arduino-cli compile --fqbn arduino:avr:mega arduino_improved/e38s_encoder_improved
/home/x4/bin/arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega arduino_improved/e38s_encoder_improved
```

If you're getting "command not found" errors, try one of these solutions:
1. Start a new terminal session
2. Manually update your PATH:
   ```bash
   export PATH=$PATH:/home/x4/bin
   ```
3. Always use the full path as shown above

Remember to upload the `e38s_encoder_improved.ino` sketch to your Arduino after completing the wiring.
