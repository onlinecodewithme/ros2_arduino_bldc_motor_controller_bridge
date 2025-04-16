/*
 * ROS2 Compatible Arduino Motor Controller with E38s6G5 Encoders
 *
 * This sketch provides a reliable bridge between ROS2 and Arduino-controlled motors
 * with support for E38s6G5 1000B G24N rotary encoders.
 */

#include <Arduino.h>  // Include Arduino core functionalities
#include <string.h>   // For memcpy function

// Pin definitions for motor control
const int leftThrottlePin = 9;   // PWM output to left throttle
const int rightThrottlePin = 10; // PWM output to right throttle
const int leftDirPin = 7;        // Left direction control
const int rightDirPin = 8;       // Right direction control

// Pin definitions for E38s6G5 1000B G24N encoders
const int leftEncoderPinA = 2;   // Left encoder channel A (must be interrupt pin)
const int leftEncoderPinB = 3;   // Left encoder channel B (must be interrupt pin)
const int rightEncoderPinA = 18; // Right encoder channel A (must be interrupt pin)
const int rightEncoderPinB = 19; // Right encoder channel B (must be interrupt pin)

// Position tracking
volatile long leftPosition = 0;
volatile long rightPosition = 0;
float leftDistance = 0.0;
float rightDistance = 0.0;

// E38s6G5 encoder has 1000 PPR (pulses per revolution)
// Using both edges of both channels gives 4000 counts per revolution
const float ticksPerRevolution = 4000.0;
const float wheelDiameter = 0.165;  // in meters, adjust to your wheel
const float metersPerTick = (PI * wheelDiameter) / ticksPerRevolution;

// Movement parameters
int baseSpeed = 70;           // Base speed set to 70 as requested
bool moving = false;
int currentSpeed = 70;        // Set current speed to 70
int minMotorSpeed = 70;       // Minimum speed needed to rotate the motors reliably
int minBackwardSpeed = 85;    // Higher minimum for backward motion (adjusted proportionally)

// Motor trim (fixed values - keeps it simple and stable)
const float leftMotorTrim = 1.05;   // Left motor gets 5% more power
const float rightMotorTrim = 1.0;   // Right motor at normal power

// Serial communication
const long baudRate = 115200;
unsigned long lastOdomTime = 0;
const unsigned long odomPeriod = 100;
const unsigned long serialTimeout = 50;

// Watchdog timer
unsigned long lastCommandTime = 0;
const unsigned long watchdogTimeout = 3000;

// ROS2 protocol constants
const char COMMAND_HEADER = '#';
const char VELOCITY_HEADER = 'V';
const char ODOMETRY_HEADER = 'O';
const char RESET_HEADER = 'R';
const char ACK_HEADER = 'A';
const char DEBUG_HEADER = 'D';

void setup() {
  // Setup motor control pins
  pinMode(leftThrottlePin, OUTPUT);
  pinMode(rightThrottlePin, OUTPUT);
  pinMode(leftDirPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  
  // Setup encoder pins with pull-up resistors
  pinMode(leftEncoderPinA, INPUT_PULLUP);
  pinMode(leftEncoderPinB, INPUT_PULLUP);
  pinMode(rightEncoderPinA, INPUT_PULLUP);
  pinMode(rightEncoderPinB, INPUT_PULLUP);
  
  // Attach interrupts for quadrature encoders
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinB), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinB), rightEncoderISR, CHANGE);
  
  // Initial state - stopped
  stopMotors();
  
  // Start serial communication
  Serial.begin(baudRate);
  delay(1000);  // Allow time for serial to initialize
  
  Serial.println("ROS2 Motor Controller Initialized - E38s ENCODER VERSION");
  printMenu();
}

void loop() {
  // Check for serial commands
  processSerial();
  
  // Periodically send odometry data
  unsigned long currentTime = millis();
  if (currentTime - lastOdomTime >= odomPeriod) {
    sendOdometry();
    lastOdomTime = currentTime;
  }
  
  // Safety watchdog - stop motors if no commands received for a while
  if (moving && (currentTime - lastCommandTime) > watchdogTimeout) {
    stopMotors();
    Serial.println("Watchdog timeout - motors stopped");
  }
}

// Encoder Interrupt Service Routines using proper quadrature decoding
void leftEncoderISR() {
  static byte lastLeftState = 0;
  
  // Read the current state of both encoder pins
  byte leftState = (digitalRead(leftEncoderPinA) << 1) | digitalRead(leftEncoderPinB);
  
  // Determine direction based on the state change
  switch (lastLeftState) {
    case 0:
      if (leftState == 1) leftPosition++;
      if (leftState == 2) leftPosition--;
      break;
    case 1:
      if (leftState == 3) leftPosition++;
      if (leftState == 0) leftPosition--;
      break;
    case 2:
      if (leftState == 0) leftPosition++;
      if (leftState == 3) leftPosition--;
      break;
    case 3:
      if (leftState == 2) leftPosition++;
      if (leftState == 1) leftPosition--;
      break;
  }
  
  lastLeftState = leftState;
}

void rightEncoderISR() {
  static byte lastRightState = 0;
  
  // Read the current state of both encoder pins
  byte rightState = (digitalRead(rightEncoderPinA) << 1) | digitalRead(rightEncoderPinB);
  
  // Determine direction based on the state change
  switch (lastRightState) {
    case 0:
      if (rightState == 1) rightPosition++;
      if (rightState == 2) rightPosition--;
      break;
    case 1:
      if (rightState == 3) rightPosition++;
      if (rightState == 0) rightPosition--;
      break;
    case 2:
      if (rightState == 0) rightPosition++;
      if (rightState == 3) rightPosition--;
      break;
    case 3:
      if (rightState == 2) rightPosition++;
      if (rightState == 1) rightPosition--;
      break;
  }
  
  lastRightState = rightState;
}

void processSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Process special headers that expect binary data
    if (c == VELOCITY_HEADER) {
      processVelocityCommand();
      lastCommandTime = millis();
      continue;
    }
    else if (c == RESET_HEADER) {
      resetOdometry();
      Serial.println("AOdometry reset");
      continue;
    }
    else if (c == DEBUG_HEADER) {
      sendDebugInfo();
      continue;
    }
    
    // For ASCII commands process directly
    if (c == COMMAND_HEADER) {
      // Wait for next character
      if (waitForSerial()) {
        processCommand(Serial.read());
        lastCommandTime = millis();
      }
    }
    // Other direct commands (like wasdx)
    else if (c >= 32 && c <= 126) {  // Printable ASCII only
      processCommand(c);
      lastCommandTime = millis();
    }
  }
}

bool waitForSerial() {
  unsigned long startTime = millis();
  while (!Serial.available()) {
    if (millis() - startTime > serialTimeout) {
      return false;
    }
  }
  return true;
}

void processVelocityCommand() {
  // Wait for all 8 bytes to be available (with timeout)
  unsigned long startTime = millis();
  while (Serial.available() < 8) {
    if (millis() - startTime > serialTimeout) {
      // Timeout occurred, flush partial data
      while (Serial.available() > 0) Serial.read();
      return;
    }
  }
  
  // Read binary float data (4 bytes each for left and right velocity)
  byte leftBytes[4];
  byte rightBytes[4];
  
  Serial.readBytes(leftBytes, 4);
  Serial.readBytes(rightBytes, 4);
  
  // Convert to floats (assuming little-endian)
  float leftVel, rightVel;
  memcpy(&leftVel, leftBytes, 4);
  memcpy(&rightVel, rightBytes, 4);
  
  // Check for NaN or extreme values
  if (isnan(leftVel) || isnan(rightVel) ||
      abs(leftVel) > 10.0 || abs(rightVel) > 10.0) {
    Serial.println("Invalid velocity values received");
    return;
  }
  
  // Process velocity command directly (no queueing)
  setMotorVelocities(leftVel, rightVel);
}

void processCommand(char cmd) {
  switch(cmd) {
    case 'w': // Forward
      moveForward(currentSpeed);
      break;
    case 's': // Backward
      moveBackward(currentSpeed);
      break;
    case 'a': // Left turn
      turnLeft(currentSpeed);
      break;
    case 'd': // Right turn
      turnRight(currentSpeed);
      break;
    case 'q': // Pivot left
      pivotLeft(currentSpeed);
      break;
    case 'e': // Pivot right
      pivotRight(currentSpeed);
      break;
    case 'x': // Stop
      stopMotors();
      break;
    case '+': // Increase speed
      currentSpeed = min(255, currentSpeed + 10);
      updateMovement();
      break;
    case '-': // Decrease speed
      currentSpeed = max(minMotorSpeed, currentSpeed - 10);
      updateMovement();
      break;
    case '?': // Print menu
      printMenu();
      break;
    default:
      // Ignore other characters
      break;
  }
}

void printMenu() {
  Serial.println("\n=== ROS2 Motor Controller - E38s ENCODER VERSION ===");
  Serial.println("Manual Controls:");
  Serial.println("  w - Move forward");
  Serial.println("  s - Move backward");
  Serial.println("  a - Turn left");
  Serial.println("  d - Turn right");
  Serial.println("  q - Pivot left");
  Serial.println("  e - Pivot right");
  Serial.println("  x - Stop");
  Serial.println("  + - Increase speed");
  Serial.println("  - - Decrease speed");
  Serial.println("  ? - Show this menu");
  Serial.print("Current speed: ");
  Serial.println(currentSpeed);
}

void updateMovement() {
  // Re-apply the current movement with the new speed
  if (moving) {
    // Detect current state based on pin values
    bool leftDir = digitalRead(leftDirPin);
    bool rightDir = digitalRead(rightDirPin);
    
    if (leftDir == HIGH && rightDir == HIGH) {
      moveForward(currentSpeed);
    } else if (leftDir == LOW && rightDir == LOW) {
      moveBackward(currentSpeed);
    } else if (leftDir == LOW && rightDir == HIGH) {
      turnLeft(currentSpeed);
    } else if (leftDir == HIGH && rightDir == LOW) {
      turnRight(currentSpeed);
    }
  }
}

// Basic movement functions with fixed trim adjustment
void moveForward(int speed) {
  digitalWrite(leftDirPin, HIGH);   // Forward direction (SWAPPED)
  digitalWrite(rightDirPin, HIGH);  // Forward direction (SWAPPED)
  analogWrite(leftThrottlePin, speed * leftMotorTrim);
  analogWrite(rightThrottlePin, speed * rightMotorTrim);
  moving = true;
}

void moveBackward(int speed) {
  digitalWrite(leftDirPin, LOW);    // Backward direction (SWAPPED)
  digitalWrite(rightDirPin, LOW);   // Backward direction (SWAPPED)
  analogWrite(leftThrottlePin, speed * leftMotorTrim);
  analogWrite(rightThrottlePin, speed * rightMotorTrim);
  moving = true;
}

void turnLeft(int speed) {
  digitalWrite(leftDirPin, LOW);    // Left track backward (SWAPPED)
  digitalWrite(rightDirPin, HIGH);  // Right track forward (SWAPPED)
  // For turns, inner wheel runs slower than outer wheel
  float innerWheelFactor = 0.7;
  analogWrite(leftThrottlePin, speed * leftMotorTrim * innerWheelFactor);
  analogWrite(rightThrottlePin, speed * rightMotorTrim);
  moving = true;
}

void turnRight(int speed) {
  digitalWrite(leftDirPin, HIGH);   // Left track forward (SWAPPED)
  digitalWrite(rightDirPin, LOW);   // Right track backward (SWAPPED)
  // For turns, inner wheel runs slower than outer wheel
  float innerWheelFactor = 0.7;
  analogWrite(leftThrottlePin, speed * leftMotorTrim);
  analogWrite(rightThrottlePin, speed * rightMotorTrim * innerWheelFactor);
  moving = true;
}

void pivotLeft(int speed) {
  digitalWrite(leftDirPin, LOW);    // Left track backward (SWAPPED)
  digitalWrite(rightDirPin, HIGH);  // Right track forward (SWAPPED)
  analogWrite(leftThrottlePin, speed * leftMotorTrim);
  analogWrite(rightThrottlePin, speed * rightMotorTrim);
  moving = true;
}

void pivotRight(int speed) {
  digitalWrite(leftDirPin, HIGH);   // Left track forward (SWAPPED)
  digitalWrite(rightDirPin, LOW);   // Right track backward (SWAPPED)
  analogWrite(leftThrottlePin, speed * leftMotorTrim);
  analogWrite(rightThrottlePin, speed * rightMotorTrim);
  moving = true;
}

void stopMotors() {
  analogWrite(leftThrottlePin, 0);
  analogWrite(rightThrottlePin, 0);
  // Keep direction pins in their current state
  moving = false;
}

// Simplified motor velocity control
void setMotorVelocities(float leftVel, float rightVel) {
  // Track previous directions to detect direction changes
  static bool prevLeftDir = HIGH;  // Forward by default (SWAPPED)
  static bool prevRightDir = HIGH; // Forward by default (SWAPPED)
  
  // Convert m/s velocities to motor speeds and directions
  bool newLeftDir = leftVel < 0 ? LOW : HIGH;   // LOW for backward (SWAPPED)
  bool newRightDir = rightVel < 0 ? LOW : HIGH; // LOW for backward (SWAPPED)
  
  // Select appropriate minimum speed based on direction
  int leftMinSpeed = (leftVel < 0) ? minBackwardSpeed : minMotorSpeed;
  int rightMinSpeed = (rightVel < 0) ? minBackwardSpeed : minMotorSpeed;
  
  // Calculate speeds with deadband
  int leftSpeed = 0, rightSpeed = 0;
  
  if (abs(leftVel) < 0.01) {
    leftSpeed = 0;
  } else {
    // Simple linear mapping
    leftSpeed = leftMinSpeed + (abs(leftVel) * (255.0 - leftMinSpeed));
    leftSpeed = constrain(leftSpeed, leftMinSpeed, 255);
  }
  
  if (abs(rightVel) < 0.01) {
    rightSpeed = 0;
  } else {
    // Simple linear mapping
    rightSpeed = rightMinSpeed + (abs(rightVel) * (255.0 - rightMinSpeed));
    rightSpeed = constrain(rightSpeed, rightMinSpeed, 255);
  }
  
  // Check for direction changes
  bool directionChange = false;
  
  if ((newLeftDir != prevLeftDir && leftSpeed > 0) || 
      (newRightDir != prevRightDir && rightSpeed > 0)) {
    directionChange = true;
    
    // First stop motors completely
    analogWrite(leftThrottlePin, 0);
    analogWrite(rightThrottlePin, 0);
    delay(250);  // Extra long delay to ensure complete stop
    
    // Set new directions
    digitalWrite(leftDirPin, newLeftDir);
    digitalWrite(rightDirPin, newRightDir);
    delay(200);  // Wait for direction change to settle
    
    // Special handling for backward direction
    if (newLeftDir == HIGH || newRightDir == HIGH) {
      // For backward motion, start with higher speed pulse
      analogWrite(leftThrottlePin, leftSpeed > 0 ? max(leftSpeed, 200) : 0);
      analogWrite(rightThrottlePin, rightSpeed > 0 ? max(rightSpeed, 200) : 0);
      delay(100);  // Brief high-power pulse
      
      // Then drop to minimum speed
      analogWrite(leftThrottlePin, leftSpeed > 0 ? leftMinSpeed : 0);
      analogWrite(rightThrottlePin, rightSpeed > 0 ? rightMinSpeed : 0);
      delay(150);
    } else {
      // For forward motion, start slowly
      analogWrite(leftThrottlePin, leftSpeed > 0 ? leftMinSpeed : 0);
      analogWrite(rightThrottlePin, rightSpeed > 0 ? rightMinSpeed : 0);
      delay(150);
    }
  } else {
    // No direction change, just update direction pins
    digitalWrite(leftDirPin, newLeftDir);
    digitalWrite(rightDirPin, newRightDir);
  }
  
  // Apply motor trims to the final speeds
  int finalLeftSpeed = leftSpeed > 0 ? leftSpeed * leftMotorTrim : 0;
  int finalRightSpeed = rightSpeed > 0 ? rightSpeed * rightMotorTrim : 0;
  
  // Ensure speeds are in valid range
  finalLeftSpeed = constrain(finalLeftSpeed, 0, 255);
  finalRightSpeed = constrain(finalRightSpeed, 0, 255);
  
  // Apply the final speeds - use a ramp if there was a direction change
  if (directionChange) {
    // Ramp up to final speed
    int steps = 3;
    for (int i = 1; i <= steps; i++) {
      int leftStepSpeed = leftMinSpeed + i * (finalLeftSpeed - leftMinSpeed) / steps;
      int rightStepSpeed = rightMinSpeed + i * (finalRightSpeed - rightMinSpeed) / steps;
      
      if (leftSpeed > 0) analogWrite(leftThrottlePin, leftStepSpeed);
      if (rightSpeed > 0) analogWrite(rightThrottlePin, rightStepSpeed);
      delay(70);
    }
  }
  
  // Set final speed
  analogWrite(leftThrottlePin, finalLeftSpeed);
  analogWrite(rightThrottlePin, finalRightSpeed);
  
  // Store current directions for next call
  prevLeftDir = newLeftDir;
  prevRightDir = newRightDir;
  
  // Update movement state
  moving = (finalLeftSpeed > 0 || finalRightSpeed > 0);
  
  // Update current speed for manual control
  currentSpeed = max(finalLeftSpeed, finalRightSpeed);
}

void sendOdometry() {
  // Calculate distance in meters
  leftDistance = leftPosition * metersPerTick;
  rightDistance = rightPosition * metersPerTick;
  
  // Format: O left_pos right_pos left_dist right_dist
  Serial.print(ODOMETRY_HEADER);
  Serial.print(leftPosition);
  Serial.print(" ");
  Serial.print(rightPosition);
  Serial.print(" ");
  Serial.print(leftDistance, 6);  // 6 decimal places
  Serial.print(" ");
  Serial.println(rightDistance, 6);
}

void resetOdometry() {
  noInterrupts();  // Disable interrupts during reset
  leftPosition = 0;
  rightPosition = 0;
  leftDistance = 0.0;
  rightDistance = 0.0;
  interrupts();    // Re-enable interrupts
}

void sendDebugInfo() {
  // Send debug information about current pin states
  Serial.print(DEBUG_HEADER);
  Serial.print("Moving:");
  Serial.print(moving);
  Serial.print(" Speed:");
  Serial.print(currentSpeed);
  Serial.print(" LeftDir:");
  Serial.print(digitalRead(leftDirPin));
  Serial.print(" RightDir:");
  Serial.print(digitalRead(rightDirPin));
  Serial.print(" LeftPos:");
  Serial.print(leftPosition);
  Serial.print(" RightPos:");
  Serial.println(rightPosition);
}
