# E38S Encoder Setup Guide

This guide provides instructions for setting up and connecting E38S high-resolution encoders with your Arduino-based motor control system.

## E38S Encoder Specifications

- **Resolution**: 4000 pulses per revolution (11x more precise than hall sensors)
- **Supply Voltage**: 5-24V DC
- **Output Signal**: Quadrature (A, B) with optional Z (index)
- **Maximum Speed**: 6000 RPM
- **IP Rating**: IP50 (protected against dust)
- **Shaft Diameter**: 6mm

## Wiring Connections

Connect the E38S encoders to your Arduino Mega as follows:

### Left Encoder

| E38S Wire | Color  | Arduino Connection |
|-----------|--------|-------------------|
| VCC       | Red    | 5V                |
| GND       | Black  | GND               |
| A         | Yellow | Digital Pin 2     |
| B         | Green  | Digital Pin 3     |
| Z (Index) | Blue   | Digital Pin 18    |

### Right Encoder

| E38S Wire | Color  | Arduino Connection |
|-----------|--------|-------------------|
| VCC       | Red    | 5V                |
| GND       | Black  | GND               |
| A         | Yellow | Digital Pin 19    |
| B         | Green  | Digital Pin 20    |
| Z (Index) | Blue   | Digital Pin 21    |

## Arduino Code Modifications

Add the following code to your Arduino sketch to handle E38S encoder readings:

```cpp
// Encoder pins
#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 3
#define LEFT_ENCODER_Z 18
#define RIGHT_ENCODER_A 19
#define RIGHT_ENCODER_B 20
#define RIGHT_ENCODER_Z 21

// Encoder counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

void setup() {
  // Set up encoder pins
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_Z, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_Z, INPUT_PULLUP);
  
  // Attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, CHANGE);
}

// Left encoder interrupt service routine
void leftEncoderISR() {
  // If A leads B, increment; else decrement
  if (digitalRead(LEFT_ENCODER_A) == digitalRead(LEFT_ENCODER_B)) {
    leftEncoderCount--;
  } else {
    leftEncoderCount++;
  }
}

// Right encoder interrupt service routine
void rightEncoderISR() {
  // If A leads B, increment; else decrement
  if (digitalRead(RIGHT_ENCODER_A) == digitalRead(RIGHT_ENCODER_B)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}
```

## Physical Installation

1. Mount the encoders directly to the motor shafts using couplers
2. Ensure the encoders are properly aligned with the motor shafts
3. Secure the encoder body to prevent rotation
4. Use shielded cables for encoder connections to reduce noise
5. Keep wiring away from motor power wires to minimize electromagnetic interference

## Troubleshooting

### No Encoder Readings

- Check power connections (5V and GND)
- Verify that encoder wires are properly connected to the correct pins
- Check if encoder shaft is properly coupled to the motor shaft

### Inconsistent Readings

- Check for loose connections
- Ensure encoder mounting is stable and not moving
- Verify that the encoder shaft is not slipping on the motor shaft
- Check for electromagnetic interference from nearby motors or power lines

### Inverted Direction

If the encoder direction is opposite to what you expect, swap the A and B connections or invert the logic in your code.

## Running the Improved Navigation with E38S Encoders

Use the robust encoder bridge launch script:

```bash
./src/robot_navigation/run_robust_encoder_navigation.sh
```

This script provides:
- Robust serial reconnection
- Automatic serial port release if already in use
- Support for high-resolution encoders
- Improved odometry for better navigation

## Additional Parameters

You can customize the encoder setup with additional parameters:

```bash
./src/robot_navigation/run_robust_encoder_navigation.sh --wheel-radius=0.0825 --base-width=0.17 --ticks-per-rev=4000.0
