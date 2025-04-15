/*
 * Improved ROS2 Compatible Arduino Motor Controller
 *
 * This sketch provides a bridge between ROS2 and Arduino-controlled motors.
 * It addresses communication and timing issues in the original code
 * for a more reliable connection with the ROS2 bridge.
 */

// Pin definitions
const int leftThrottlePin = 9;   // PWM output to left throttle
const int rightThrottlePin = 10; // PWM output to right throttle
const int leftDirPin = 7;        // Left direction control
const int rightDirPin = 8;       // Right direction control
const int leftHallPin = 2;       // Left hall sensor (using just one for simplicity)
const int rightHallPin = 5;      // Right hall sensor (using just one for simplicity)

// Position tracking
volatile long leftPosition = 0;
volatile long rightPosition = 0;
// Distance tracking (in encoder ticks converted to meters)
float leftDistance = 0.0;
float rightDistance = 0.0;
// Wheel parameters for distance calculation
const float ticksPerRevolution = 360.0;  // Adjust to your encoder resolution
const float wheelDiameter = 0.165;       // Wheel diameter in meters (adjust to your wheel size)
const float metersPerTick = (PI * wheelDiameter) / ticksPerRevolution;

// Movement parameters
int baseSpeed = 150;      // Base movement speed (0-255)
bool moving = false;      // Movement state
int currentSpeed = 150;   // Current speed value (0-255)
int minMotorSpeed = 90;   // Minimum speed needed to rotate the motors reliably
int minBackwardSpeed = 110; // Higher minimum speed for backward motion

// Serial communication
const long baudRate = 115200;
unsigned long lastOdomTime = 0;
const unsigned long odomPeriod = 100;  // Send odometry every 100ms
char serialBuffer[64];
int bufferIndex = 0;
const unsigned long serialTimeout = 50; // ms to wait for serial data

// Activity monitoring
unsigned long lastCommandTime = 0;
const unsigned long watchdogTimeout = 3000; // Stop motors if no commands for 3 seconds

// ROS2 protocol constants
const char COMMAND_HEADER = '#';
const char VELOCITY_HEADER = 'V';
const char ODOMETRY_HEADER = 'O';
const char RESET_HEADER = 'R';
const char ACK_HEADER = 'A';
const char DEBUG_HEADER = 'D';

void setup() {
  // Setup pins
  pinMode(leftThrottlePin, OUTPUT);
  pinMode(rightThrottlePin, OUTPUT);
  pinMode(leftDirPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);

  // Initialize hall sensor pins and attach interrupts
  pinMode(leftHallPin, INPUT_PULLUP);
  pinMode(rightHallPin, INPUT_PULLUP);

  // Attach interrupts for position tracking
  attachInterrupt(digitalPinToInterrupt(leftHallPin), leftHallChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightHallPin), rightHallChange, CHANGE);

  // Initial state - stopped
  stopMotors();

  // Start serial communication
  Serial.begin(baudRate);

  // Allow time for serial to initialize (not waiting for connection)
  delay(500);

  // Send initialization message
  Serial.println("ROS2 Motor Controller Initialized v2.0");
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
      // Timeout occurred flush partial data
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

  // Process velocity command
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
  Serial.println("\n=== ROS2 Motor Controller v2.0 ===");
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

    if (leftDir == LOW && rightDir == LOW) {
      moveForward(currentSpeed);
    } else if (leftDir == HIGH && rightDir == HIGH) {
      moveBackward(currentSpeed);
    } else if (leftDir == HIGH && rightDir == LOW) {
      turnLeft(currentSpeed);
    } else if (leftDir == LOW && rightDir == HIGH) {
      turnRight(currentSpeed);
    }
  }
}

// Basic movement functions
void moveForward(int speed) {
  digitalWrite(leftDirPin, LOW);    // Forward direction
  digitalWrite(rightDirPin, LOW);   // Forward direction
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void moveBackward(int speed) {
  digitalWrite(leftDirPin, HIGH);   // Backward direction
  digitalWrite(rightDirPin, HIGH);  // Backward direction
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void turnLeft(int speed) {
  digitalWrite(leftDirPin, HIGH);   // Left track backward
  digitalWrite(rightDirPin, LOW);   // Right track forward
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void turnRight(int speed) {
  digitalWrite(leftDirPin, LOW);    // Left track forward
  digitalWrite(rightDirPin, HIGH);  // Right track backward
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void pivotLeft(int speed) {
  digitalWrite(leftDirPin, HIGH);   // Left track backward
  digitalWrite(rightDirPin, LOW);   // Right track forward
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void pivotRight(int speed) {
  digitalWrite(leftDirPin, LOW);    // Left track forward
  digitalWrite(rightDirPin, HIGH);  // Right track backward
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void stopMotors() {
  analogWrite(leftThrottlePin, 0);
  analogWrite(rightThrottlePin, 0);
  // Keep direction pins in their current state
  moving = false;
}

// ROS2 interface functions
void setMotorVelocities(float leftVel, float rightVel) {
  // Track previous directions to detect direction changes
  static bool prevLeftDir = LOW;  // Forward by default
  static bool prevRightDir = LOW; // Forward by default

  // Convert m/s velocities to motor speeds and directions
  // Use different minimum speeds for forward and backward
  bool newLeftDir = leftVel < 0 ? HIGH : LOW;   // HIGH for backward
  bool newRightDir = rightVel < 0 ? HIGH : LOW; // HIGH for backward
  
  // Select appropriate minimum speed based on direction
  int leftMinSpeed = (leftVel < 0) ? minBackwardSpeed : minMotorSpeed;
  int rightMinSpeed = (rightVel < 0) ? minBackwardSpeed : minMotorSpeed;
  
  // Apply constraints
  int leftSpeed = abs(leftVel) < 0.01 ? 0 : constrain(abs(leftVel) * 255.0, leftMinSpeed, 255);
  int rightSpeed = abs(rightVel) < 0.01 ? 0 : constrain(abs(rightVel) * 255.0, rightMinSpeed, 255);
  
  // If there's a direction change, smoothly transition
  if ((newLeftDir != prevLeftDir && leftSpeed > 0) || 
      (newRightDir != prevRightDir && rightSpeed > 0)) {
    
    // First stop motors completely
    analogWrite(leftThrottlePin, 0);
    analogWrite(rightThrottlePin, 0);
    delay(150);  // Longer delay to ensure complete stop
    
    // Set new directions
    digitalWrite(leftDirPin, newLeftDir);
    digitalWrite(rightDirPin, newRightDir);
    delay(100);  // Longer pause after direction change
    
    // Special handling for backward direction
    if (newLeftDir == HIGH || newRightDir == HIGH) {
      // For backward motion, start with higher speed pulse
      analogWrite(leftThrottlePin, leftSpeed > 0 ? max(leftSpeed, 150) : 0);
      analogWrite(rightThrottlePin, rightSpeed > 0 ? max(rightSpeed, 150) : 0);
      delay(50);  // Brief pulse
      
      // Drop to minimum speed
      analogWrite(leftThrottlePin, leftSpeed > 0 ? leftMinSpeed : 0);
      analogWrite(rightThrottlePin, rightSpeed > 0 ? rightMinSpeed : 0);
      delay(100);
    } else {
      // For forward motion, start slowly
      analogWrite(leftThrottlePin, leftSpeed > 0 ? leftMinSpeed : 0);
      analogWrite(rightThrottlePin, rightSpeed > 0 ? rightMinSpeed : 0);
      delay(100);
    }
    
    // Then gradually increase to half speed
    analogWrite(leftThrottlePin, leftSpeed > 0 ? leftSpeed/2 : 0);
    analogWrite(rightThrottlePin, rightSpeed > 0 ? rightSpeed/2 : 0);
    delay(100);
  } else {
    // No direction change, just update direction pins
    digitalWrite(leftDirPin, newLeftDir);
    digitalWrite(rightDirPin, newRightDir);
  }

  // Set final speeds
  analogWrite(leftThrottlePin, leftSpeed);
  analogWrite(rightThrottlePin, rightSpeed);

  // Store current directions for next call
  prevLeftDir = newLeftDir;
  prevRightDir = newRightDir;

  // Update movement state
  moving = (leftSpeed > 0 || rightSpeed > 0);

  // Update current speed for manual control
  currentSpeed = max(leftSpeed, rightSpeed);
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
  Serial.print(" LeftThrottle:");
  Serial.print(analogRead(leftThrottlePin));
  Serial.print(" RightThrottle:");
  Serial.print(analogRead(rightThrottlePin));
  Serial.print(" LeftPos:");
  Serial.print(leftPosition);
  Serial.print(" RightPos:");
  Serial.println(rightPosition);
}

// Interrupt handlers for position tracking
void leftHallChange() {
  // Update position based on direction
  if (digitalRead(leftDirPin) == LOW) {
    leftPosition++;
  } else {
    leftPosition--;
  }
}

void rightHallChange() {
  // Update position based on direction
  if (digitalRead(rightDirPin) == LOW) {
    rightPosition++;
  } else {
    rightPosition--;
  }
}
