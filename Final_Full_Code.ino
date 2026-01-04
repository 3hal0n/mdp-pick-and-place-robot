/* ME2610 Wall Following Mobile Robot - Enhanced Version
 * Features: PID Control, Sensor Filtering, Multi-pass Alignment,
 *           Path Memory, Improved Accuracy
 */

#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// ============ PIN DEFINITIONS ============
// Stepper Motors (using A4988 drivers)
#define MOTOR_LEFT_STEP 2
#define MOTOR_LEFT_DIR 5
#define MOTOR_RIGHT_STEP 3
#define MOTOR_RIGHT_DIR 6

// Ultrasonic Sensors
#define US_LEFT_TRIG 22
#define US_LEFT_ECHO 23
#define US_RIGHT_TRIG 24
#define US_RIGHT_ECHO 25
#define US_RADAR_UPPER_TRIG 26
#define US_RADAR_UPPER_ECHO 27
#define US_RADAR_LOWER_TRIG 28
#define US_RADAR_LOWER_ECHO 29
#define US_GRIPPER_CENTER_TRIG 30
#define US_GRIPPER_CENTER_ECHO 31

// IR Sensor (Rear)
#define IR_REAR A0

// Servos
#define SERVO_RADAR 6
#define SERVO_GRIPPER 7
#define SERVO_TILT 8

// Color Sensor (TCS3200)
#define COLOR_S0 9
#define COLOR_S1 10
#define COLOR_S2 11
#define COLOR_S3 12
#define COLOR_OUT 13

// Buttons
#define BUTTON_START 14
#define BUTTON_STOP 15

// Battery Monitoring (optional)
#define BATTERY_VOLTAGE A1

// ============ CALIBRATION CONSTANTS ============
// IMPORTANT: Measure these values for your specific robot!
#define STEPS_PER_REV 200
#define WHEEL_DIAMETER 6.5      // cm - MEASURE ACTUAL WHEEL
#define WHEEL_BASE 20.0         // cm - MEASURE ACTUAL DISTANCE BETWEEN WHEELS
#define STEPS_PER_CM ((STEPS_PER_REV) / (PI * WHEEL_DIAMETER))

// Navigation Constants
#define WALL_FOLLOW_DISTANCE 15  // cm from left wall
#define CUBE_HEIGHT_DIFF 5       // cm height difference threshold
#define GRIP_DISTANCE 8          // cm distance to grip cube

// Safety Distances
#define EMERGENCY_FRONT_DIST 8   // cm
#define EMERGENCY_SIDE_DIST 3    // cm
#define EMERGENCY_REAR_DIST 5    // cm

// Alignment Tolerances
#define COARSE_TOLERANCE 3.0     // cm - first pass
#define FINE_TOLERANCE 1.0       // cm - second pass

// Timeout Values
#define MAX_ALIGNMENT_ATTEMPTS 10
#define MAX_GRIP_ATTEMPTS 3
#define STATE_TIMEOUT 30000      // 30 seconds per state max

// ============ PID CONSTANTS ============
// Wall Following PID (tune these for your robot)
#define KP_WALL 0.8    // Proportional gain
#define KI_WALL 0.1    // Integral gain
#define KD_WALL 0.3    // Derivative gain

// ============ GLOBAL OBJECTS ============
AccelStepper motorLeft(AccelStepper::DRIVER, MOTOR_LEFT_STEP, MOTOR_LEFT_DIR);
AccelStepper motorRight(AccelStepper::DRIVER, MOTOR_RIGHT_STEP, MOTOR_RIGHT_DIR);
MultiStepper steppers;

Servo servoRadar;
Servo servoGripper;
Servo servoTilt;

// ============ STATE MACHINE ============
enum RobotState {
  SEARCHING_CUBE,
  APPROACHING_CUBE,
  ALIGNING_TO_CUBE,
  GRIPPING_CUBE,
  SEARCHING_COLOR,
  PLACING_CUBE
};

RobotState currentState = SEARCHING_CUBE;
unsigned long stateStartTime = 0;

// ============ POSITION TRACKING (ODOMETRY) ============
float robotX = 0;
float robotY = 0;
float robotAngle = 0;  // 0=right, 90=up, 180=left, 270=down

// ============ CUBE MEMORY SYSTEM ============
struct CubePosition {
  float x;
  float y;
  bool placed;
  unsigned long timestamp;
};
CubePosition placedCubes[5];
int placedCubeCount = 0;

// Path Memory - stores visited positions to avoid redundant movement
struct PathNode {
  float x;
  float y;
  unsigned long timestamp;
};
PathNode visitedPath[100];
int pathNodeCount = 0;

// Color tracking
bool colorPlaced[3] = {false, false, false};  // red, blue, green
int cubesPlaced = 0;

// ============ DETECTION VARIABLES ============
int detectedCubeAngle = 0;
float detectedCubeDistance = 0;

// ============ PID VARIABLES ============
float wallPID_error = 0;
float wallPID_lastError = 0;
float wallPID_integral = 0;
unsigned long lastPIDTime = 0;

// ============ SENSOR FILTERING ============
#define FILTER_SIZE 5
float leftDistBuffer[FILTER_SIZE];
float rightDistBuffer[FILTER_SIZE];
float rearDistBuffer[FILTER_SIZE];
int filterIndex = 0;

// ============ SETUP ============
void setup() {
  Serial.begin(115200);
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║  ME2610 Wall Following Robot - Enhanced║");
  Serial.println("╚════════════════════════════════════════╝");
  
  // Configure stepper motors with acceleration
  motorLeft.setMaxSpeed(1000);
  motorLeft.setAcceleration(500);   // Smooth acceleration
  motorRight.setMaxSpeed(1000);
  motorRight.setAcceleration(500);
  
  // Add motors to MultiStepper for synchronized movement
  steppers.addStepper(motorLeft);
  steppers.addStepper(motorRight);
  
  // Configure servos
  servoRadar.attach(SERVO_RADAR);
  servoGripper.attach(SERVO_GRIPPER);
  servoTilt.attach(SERVO_TILT);
  
  // Initial servo positions
  servoRadar.write(90);
  servoGripper.write(90);    // open
  servoTilt.write(90);       // horizontal
  
  // Configure sensor pins
  pinMode(US_LEFT_TRIG, OUTPUT);
  pinMode(US_LEFT_ECHO, INPUT);
  pinMode(US_RIGHT_TRIG, OUTPUT);
  pinMode(US_RIGHT_ECHO, INPUT);
  pinMode(US_RADAR_UPPER_TRIG, OUTPUT);
  pinMode(US_RADAR_UPPER_ECHO, INPUT);
  pinMode(US_RADAR_LOWER_TRIG, OUTPUT);
  pinMode(US_RADAR_LOWER_ECHO, INPUT);
  pinMode(US_GRIPPER_CENTER_TRIG, OUTPUT);
  pinMode(US_GRIPPER_CENTER_ECHO, INPUT);
  pinMode(IR_REAR, INPUT);
  
  // Configure color sensor
  pinMode(COLOR_S0, OUTPUT);
  pinMode(COLOR_S1, OUTPUT);
  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  digitalWrite(COLOR_S0, HIGH);
  digitalWrite(COLOR_S1, LOW);  // 20% frequency scaling
  
  // Configure buttons
  pinMode(BUTTON_START, INPUT_PULLUP);
  pinMode(BUTTON_STOP, INPUT_PULLUP);
  
  // Initialize sensor buffers
  for (int i = 0; i < FILTER_SIZE; i++) {
    leftDistBuffer[i] = WALL_FOLLOW_DISTANCE;
    rightDistBuffer[i] = WALL_FOLLOW_DISTANCE;
    rearDistBuffer[i] = EMERGENCY_REAR_DIST;
  }
  
  // Initialize cube memory
  for (int i = 0; i < 5; i++) {
    placedCubes[i].placed = false;
  }
  
  // Initialize path memory
  for (int i = 0; i < 100; i++) {
    visitedPath[i].timestamp = 0;
  }
  
  Serial.println("Robot initialized successfully!");
  Serial.println("Press START button to begin mission...");
  
  // Wait for start button
  while (digitalRead(BUTTON_START) == HIGH) {
    if (digitalRead(BUTTON_STOP) == LOW) {
      Serial.println("Mission aborted");
      while(1);
    }
    delay(10);
  }
  
  Serial.println("Mission started!");
  stateStartTime = millis();
  lastPIDTime = millis();
  delay(1000);
}

// ============ MAIN LOOP ============
void loop() {
  // Emergency stop check
  if (digitalRead(BUTTON_STOP) == LOW) {
    emergencyStop();
  }
  
  // Mission complete check
  if (cubesPlaced >= 3) {
    stopMotors();
    Serial.println("✓ Mission complete! 3 cubes placed.");
    while(1);
  }
  
  // State timeout check
  if (millis() - stateStartTime > STATE_TIMEOUT) {
    Serial.println("State timeout! Resetting to SEARCHING_CUBE");
    currentState = SEARCHING_CUBE;
    stateStartTime = millis();
  }
  
  // Battery monitoring (optional)
  checkBatteryVoltage();
  
  // State machine execution
  switch (currentState) {
    case SEARCHING_CUBE:
      searchForCube();
      break;
      
    case APPROACHING_CUBE:
      approachCube();
      break;
      
    case ALIGNING_TO_CUBE:
      alignToCube();
      break;
      
    case GRIPPING_CUBE:
      gripCube();
      break;
      
    case SEARCHING_COLOR:
      searchForColor();
      break;
      
    case PLACING_CUBE:
      placeCube();
      break;
  }
  
  delay(50);  // Small delay for stability
}

// ============ ENHANCED SENSOR FUNCTIONS ============

// Ultrasonic with moving average filter
float getUSDistanceFiltered(int trigPin, int echoPin, float* buffer) {
  // Take new reading
  float newReading = getUSDistanceRaw(trigPin, echoPin);
  
  // Update buffer
  buffer[filterIndex] = newReading;
  
  // Calculate moving average
  float sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += buffer[i];
  }
  
  return sum / FILTER_SIZE;
}

// Raw ultrasonic reading
float getUSDistanceRaw(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999.0;
  
  float distance = duration * 0.034 / 2;
  return distance;
}

// Median filter for critical measurements
float getUSDistanceMedian(int trigPin, int echoPin) {
  float readings[5];
  
  for (int i = 0; i < 5; i++) {
    readings[i] = getUSDistanceRaw(trigPin, echoPin);
    delay(10);
  }
  
  // Simple bubble sort
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4 - i; j++) {
      if (readings[j] > readings[j + 1]) {
        float temp = readings[j];
        readings[j] = readings[j + 1];
        readings[j + 1] = temp;
      }
    }
  }
  
  return readings[2];  // Return median
}

// Enhanced IR sensor with polynomial conversion
float getIRDistance() {
  // Take multiple readings and average
  float sum = 0;
  for (int i = 0; i < 3; i++) {
    int raw = analogRead(IR_REAR);
    float voltage = raw * (5.0 / 1023.0);
    
    // Polynomial approximation for Sharp GP2Y0A21
    float distance = 12.08 * voltage * voltage - 45.54 * voltage + 47.99;
    
    sum += distance;
    delay(10);
  }
  
  float avgDistance = sum / 3.0;
  return constrain(avgDistance, 10, 80);
}

// Update filter index
void updateFilterIndex() {
  filterIndex = (filterIndex + 1) % FILTER_SIZE;
}

// ============ PID WALL FOLLOWING ============
float calculateWallFollowingPID(float currentDistance, float targetDistance) {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastPIDTime) / 1000.0;  // Convert to seconds
  lastPIDTime = currentTime;
  
  // Calculate error
  float error = targetDistance - currentDistance;
  
  // Proportional term
  float P = KP_WALL * error;
  
  // Integral term (with anti-windup)
  wallPID_integral += error * dt;
  wallPID_integral = constrain(wallPID_integral, -10, 10);
  float I = KI_WALL * wallPID_integral;
  
  // Derivative term
  float D = 0;
  if (dt > 0) {
    D = KD_WALL * (error - wallPID_lastError) / dt;
  }
  
  wallPID_lastError = error;
  
  // Calculate output (correction angle in degrees)
  float output = P + I + D;
  
  return constrain(output, -10, 10);  // Limit correction to ±10 degrees
}

// ============ SYNCHRONIZED MOVEMENT ============
void moveForwardSync(float cm) {
  long steps = cm * STEPS_PER_CM;
  
  long positions[2];
  positions[0] = motorLeft.currentPosition() + steps;
  positions[1] = motorRight.currentPosition() + steps;
  
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  
  // Update odometry
  updateOdometry(cm, 0);
}

void turnSync(float degrees) {
  // Positive = left turn, Negative = right turn
  float arcLength = (WHEEL_BASE * PI * abs(degrees)) / 360.0;
  long steps = arcLength * STEPS_PER_CM;
  
  long positions[2];
  if (degrees > 0) {
    // Turn left
    positions[0] = motorLeft.currentPosition() - steps;
    positions[1] = motorRight.currentPosition() + steps;
  } else {
    // Turn right
    positions[0] = motorLeft.currentPosition() + steps;
    positions[1] = motorRight.currentPosition() - steps;
  }
  
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  
  // Update angle
  robotAngle += degrees;
  if (robotAngle >= 360) robotAngle -= 360;
  if (robotAngle < 0) robotAngle += 360;
  
  delay(100);  // Settling time
}

// ============ ENHANCED ODOMETRY ============
void updateOdometry(float distanceCm, float turnDegrees) {
  // Update position based on movement
  float angleRad = robotAngle * PI / 180.0;
  
  robotX += distanceCm * cos(angleRad);
  robotY += distanceCm * sin(angleRad);
  
  // Add to path memory if moved significantly
  if (distanceCm > 5 || abs(turnDegrees) > 15) {
    addToPathMemory(robotX, robotY);
  }
}

void resetOdometryAtLandmark() {
  // Called when robot reaches a known landmark (corner, colored cell)
  // This reduces accumulated drift
  Serial.println("Odometry reset at landmark");
}

// ============ PATH MEMORY SYSTEM ============
void addToPathMemory(float x, float y) {
  if (pathNodeCount >= 100) {
    // Shift array if full (remove oldest)
    for (int i = 0; i < 99; i++) {
      visitedPath[i] = visitedPath[i + 1];
    }
    pathNodeCount = 99;
  }
  
  visitedPath[pathNodeCount].x = x;
  visitedPath[pathNodeCount].y = y;
  visitedPath[pathNodeCount].timestamp = millis();
  pathNodeCount++;
}

bool isPositionRecentlyVisited(float x, float y) {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < pathNodeCount; i++) {
    // Check positions visited in last 30 seconds
    if (currentTime - visitedPath[i].timestamp < 30000) {
      float dist = sqrt(pow(x - visitedPath[i].x, 2) + pow(y - visitedPath[i].y, 2));
      if (dist < 20) {  // Within 20cm
        return true;
      }
    }
  }
  return false;
}

// ============ CUBE DETECTION ============
void searchForCube() {
  // Sweep radar and check for height difference
  for (int angle = 45; angle <= 135; angle += 5) {
    servoRadar.write(angle);
    delay(150);  // Increased settling time
    
    // Use median filter for critical detection
    float upperDist = getUSDistanceMedian(US_RADAR_UPPER_TRIG, US_RADAR_UPPER_ECHO);
    float lowerDist = getUSDistanceMedian(US_RADAR_LOWER_TRIG, US_RADAR_LOWER_ECHO);
    
    float heightDiff = abs(upperDist - lowerDist);
    
    if (heightDiff > CUBE_HEIGHT_DIFF && lowerDist < 50 && lowerDist > 5) {
      // Check if position already visited or cube already placed
      float cubeX = robotX + lowerDist * cos((robotAngle + angle - 90) * PI / 180.0);
      float cubeY = robotY + lowerDist * sin((robotAngle + angle - 90) * PI / 180.0);
      
      if (!isCubeAlreadyPlaced(cubeX, cubeY) && !isPositionRecentlyVisited(cubeX, cubeY)) {
        Serial.println("New cube detected!");
        detectedCubeAngle = angle;
        detectedCubeDistance = lowerDist;
        
        changeState(APPROACHING_CUBE);
        return;
      } else {
        Serial.println("Cube position already processed, continuing...");
      }
    }
  }
  
  // No new cube found, navigate with PID wall following
  navigateWithPID();
}

void navigateWithPID() {
  // Read filtered sensor values
  float leftDist = getUSDistanceFiltered(US_LEFT_TRIG, US_LEFT_ECHO, leftDistBuffer);
  float rightDist = getUSDistanceFiltered(US_RIGHT_TRIG, US_RIGHT_ECHO, rightDistBuffer);
  float rearDist = getIRDistance();
  
  updateFilterIndex();
  
  // Check front obstacle
  servoRadar.write(90);
  delay(50);
  float frontDist = getUSDistanceRaw(US_RADAR_UPPER_TRIG, US_RADAR_UPPER_ECHO);
  
  // Emergency stop check
  if (frontDist < EMERGENCY_FRONT_DIST || 
      leftDist < EMERGENCY_SIDE_DIST || 
      rightDist < EMERGENCY_SIDE_DIST) {
    handleObstacle(frontDist, leftDist, rightDist);
    return;
  }
  
  // PID wall following
  float correction = calculateWallFollowingPID(leftDist, WALL_FOLLOW_DISTANCE);
  
  // Apply correction and move forward
  if (abs(correction) > 1) {
    turnSync(-correction);  // Negative because left distance error needs right turn
  }
  
  moveForwardSync(5);  // Move in small increments
}

void handleObstacle(float front, float left, float right) {
  Serial.println("Obstacle detected!");
  
  if (front < EMERGENCY_FRONT_DIST) {
    // Front blocked - turn right
    Serial.println("Turning right...");
    turnSync(-90);
    
    // Check if new path is clear
    delay(200);
    servoRadar.write(90);
    delay(50);
    float newFront = getUSDistanceRaw(US_RADAR_UPPER_TRIG, US_RADAR_UPPER_ECHO);
    
    if (newFront < EMERGENCY_FRONT_DIST) {
      // Right also blocked, try left
      Serial.println("Right blocked, trying left...");
      turnSync(180);  // Turn to original left
      
      delay(200);
      newFront = getUSDistanceRaw(US_RADAR_UPPER_TRIG, US_RADAR_UPPER_ECHO);
      
      if (newFront < EMERGENCY_FRONT_DIST) {
        // All blocked, turn around
        Serial.println("All sides blocked, turning around...");
        turnSync(90);  // Complete 180 from original
      }
    }
  }
}

bool isCubeAlreadyPlaced(float x, float y) {
  for (int i = 0; i < placedCubeCount; i++) {
    if (placedCubes[i].placed) {
      float dist = sqrt(pow(x - placedCubes[i].x, 2) + pow(y - placedCubes[i].y, 2));
      if (dist < 15) {  // Increased exclusion radius
        return true;
      }
    }
  }
  return false;
}

// ============ APPROACH CUBE ============
void approachCube() {
  Serial.println("Approaching cube...");
  
  // Turn toward cube
  int angleDiff = detectedCubeAngle - 90;
  if (abs(angleDiff) > 2) {
    turnSync(angleDiff);
  }
  
  // Check distance
  servoRadar.write(90);
  delay(100);
  float frontDist = getUSDistanceMedian(US_RADAR_UPPER_TRIG, US_RADAR_UPPER_ECHO);
  
  if (frontDist > 20) {
    moveForwardSync(10);
    // Stay in approaching state
  } else {
    Serial.println("Close enough, starting alignment...");
    changeState(ALIGNING_TO_CUBE);
  }
}

// ============ MULTI-PASS ALIGNMENT ============
void alignToCube() {
  Serial.println("Aligning to cube...");
  
  // PASS 1: Coarse alignment
  Serial.println("Pass 1: Coarse alignment");
  bool coarseAligned = performAlignment(5, COARSE_TOLERANCE);
  
  if (!coarseAligned) {
    Serial.println("Coarse alignment failed, retrying approach...");
    changeState(APPROACHING_CUBE);
    return;
  }
  
  delay(200);  // Settling time between passes
  
  // PASS 2: Fine alignment
  Serial.println("Pass 2: Fine alignment");
  bool fineAligned = performAlignment(2, FINE_TOLERANCE);
  
  if (!fineAligned) {
    Serial.println("Fine alignment failed, retrying...");
    // Try one more coarse pass
    performAlignment(4, COARSE_TOLERANCE);
  }
  
  // Move to gripping distance
  servoRadar.write(90);
  delay(100);
  float finalDist = getUSDistanceMedian(US_RADAR_UPPER_TRIG, US_RADAR_UPPER_ECHO);
  
  if (finalDist > GRIP_DISTANCE + 2) {
    moveForwardSync(finalDist - GRIP_DISTANCE);
  } else if (finalDist < GRIP_DISTANCE - 2) {
    moveForwardSync(-(GRIP_DISTANCE - finalDist));  // Move back
  }
  
  changeState(GRIPPING_CUBE);
}

bool performAlignment(int adjustDegrees, float tolerance) {
  int attempts = 0;
  
  while (attempts < MAX_ALIGNMENT_ATTEMPTS) {
    // Measure at three angles
    servoRadar.write(70);
    delay(200);  // Increased settling time
    float leftDist = getUSDistanceMedian(US_RADAR_UPPER_TRIG, US_RADAR_UPPER_ECHO);
    
    servoRadar.write(110);
    delay(200);
    float rightDist = getUSDistanceMedian(US_RADAR_UPPER_TRIG, US_RADAR_UPPER_ECHO);
    
    float difference = leftDist - rightDist;
    
    Serial.print("Left: ");
    Serial.print(leftDist);
    Serial.print(" Right: ");
    Serial.print(rightDist);
    Serial.print(" Diff: ");
    Serial.println(difference);
    
    // Check if aligned
    if (abs(difference) < tolerance) {
      Serial.println("Aligned!");
      return true;
    }
    
    // Make correction
    if (leftDist < rightDist) {
      Serial.print("Adjusting RIGHT ");
      Serial.println(adjustDegrees);
      turnSync(-adjustDegrees);
    } else {
      Serial.print("Adjusting LEFT ");
      Serial.println(adjustDegrees);
      turnSync(adjustDegrees);
    }
    
    attempts++;
    delay(150);  // Settling time
  }
  
  Serial.println("Max alignment attempts reached");
  return false;
}

// ============ GRIPPER WITH RETRY LOGIC ============
void gripCube() {
  Serial.println("Attempting to grip cube...");
  
  int gripAttempts = 0;
  bool gripSuccessful = false;
  
  while (gripAttempts < MAX_GRIP_ATTEMPTS && !gripSuccessful) {
    gripAttempts++;
    Serial.print("Grip attempt ");
    Serial.println(gripAttempts);
    
    // Ensure gripper is open
    servoGripper.write(90);
    delay(500);
    
    // Close gripper slowly for better grip
    for (int pos = 90; pos >= 30; pos -= 5) {
      servoGripper.write(pos);
      delay(50);  // Slow closure
    }
    delay(500);
    
    // Lift cube slowly
    Serial.println("Lifting cube...");
    for (int pos = 90; pos <= 135; pos += 5) {
      servoTilt.write(pos);
      delay(50);  // Slow lift
    }
    delay(500);
    
    // Verify grip - check multiple times
    int verifyCount = 0;
    for (int i = 0; i < 3; i++) {
      float centerDist = getUSDistanceMedian(US_GRIPPER_CENTER_TRIG, US_GRIPPER_CENTER_ECHO);
      if (centerDist < 8) {
        verifyCount++;
      }
      delay(100);
    }
    
    if (verifyCount >= 2) {
      Serial.println("✓ Cube gripped successfully!");
      gripSuccessful = true;
    } else {
      Serial.println("✗ Grip verification failed");
      
      if (gripAttempts < MAX_GRIP_ATTEMPTS) {
        // Release and reposition
        Serial.println("Releasing and repositioning...");
        servoTilt.write(90);
        delay(300);
        servoGripper.write(90);
        delay(300);
        
        // Small adjustment
        if (gripAttempts == 2) {
          moveForwardSync(-2);  // Move back slightly
          delay(200);
          moveForwardSync(2);   // Move forward again
        }
      }
    }
  }
  
  if (gripSuccessful) {
    changeState(SEARCHING_COLOR);
  } else {
    Serial.println("Failed to grip after all attempts");
    // Release and return to searching
    servoTilt.write(90);
    delay(300);
    servoGripper.write(90);
    delay(300);
    moveForwardSync(-10);
    changeState(SEARCHING_CUBE);
  }
}

// ============ COLOR DETECTION WITH FILTERING ============
void searchForColor() {
  Serial.println("Searching for colored cell...");
  
  // Navigate while checking color
  navigateWithPID();
  
  // Check color with multiple samples
  String color = detectColorWithFiltering();
  
  if (color != "NONE") {
    int colorIndex = getColorIndex(color);
    if (colorIndex != -1 && !colorPlaced[colorIndex]) {
      Serial.print("Found ");
      Serial.print(color);
      Serial.println(" cell!");
      
      changeState(PLACING_CUBE);
    } else {
      Serial.print(color);
      Serial.println(" cell already used, continuing...");
    }
  }
}

String detectColorWithFiltering() {
  // Take 5 color readings
  int redReadings[5], greenReadings[5], blueReadings[5];
  
  for (int i = 0; i < 5; i++) {
    digitalWrite(COLOR_S2, LOW);
    digitalWrite(COLOR_S3, LOW);
    redReadings[i] = pulseIn(COLOR_OUT, LOW, 50000);
    delay(50);
    
    digitalWrite(COLOR_S2, HIGH);
    digitalWrite(COLOR_S3, HIGH);
    greenReadings[i] = pulseIn(COLOR_OUT, LOW, 50000);
    delay(50);
    
    digitalWrite(COLOR_S2, LOW);
    digitalWrite(COLOR_S3, HIGH);
    blueReadings[i] = pulseIn(COLOR_OUT, LOW, 50000);
    delay(50);
  }
  
  // Get median of each color
  int redMedian = getMedian(redReadings, 5);
  int greenMedian = getMedian(greenReadings, 5);
  int blueMedian = getMedian(blueReadings, 5);
  
  // Determine color
  int minVal = min(redMedian, min(greenMedian, blueMedian));
  
  if (redMedian == minVal && redMedian < 100) {
    return "RED";
  } else if (greenMedian == minVal && greenMedian < 100) {
    return "GREEN";
  } else if (blueMedian == minVal && blueMedian < 100) {
    return "BLUE";
  }
  
  return "NONE";
}

int getMedian(int arr[], int size) {
  // Simple bubble sort
  for (int i = 0; i < size - 1; iSContinue++) {
for (int j = 0; j < size - i - 1; j++) {
if (arr[j] > arr[j + 1]) {
int temp = arr[j];
arr[j] = arr[j + 1];
arr[j + 1] = temp;
}
}
}
return arr[size / 2];
}
int getColorIndex(String color) {
if (color == "RED") return 0;
if (color == "BLUE") return 1;
if (color == "GREEN") return 2;
return -1;
}
// ============ PLACE CUBE ============
void placeCube() {
Serial.println("Placing cube...");
stopMotors();
delay(500);
// Lower gripper slowly
for (int pos = 135; pos >= 90; pos -= 5) {
servoTilt.write(pos);
delay(50);
}
delay(500);
// Open gripper slowly
for (int pos = 30; pos <= 90; pos += 5) {
servoGripper.write(pos);
delay(50);
}
delay(500);
// Store cube position
if (placedCubeCount < 5) {
placedCubes[placedCubeCount].x = robotX;
placedCubes[placedCubeCount].y = robotY;
placedCubes[placedCubeCount].placed = true;
placedCubes[placedCubeCount].timestamp = millis();
placedCubeCount++;
}
// Mark color as used
String color = detectColorWithFiltering();
int colorIndex = getColorIndex(color);
if (colorIndex != -1) {
colorPlaced[colorIndex] = true;
}
cubesPlaced++;
Serial.print("✓ Cubes placed: ");
Serial.print(cubesPlaced);
Serial.println("/3");
// Reset odometry at landmark
resetOdometryAtLandmark();
// Back up
moveForwardSync(-10);
delay(200);
// Return to searching
changeState(SEARCHING_CUBE);
}
// ============ UTILITY FUNCTIONS ============
void stopMotors() {
motorLeft.stop();
motorRight.stop();
motorLeft.setCurrentPosition(0);
motorRight.setCurrentPosition(0);
}
void emergencyStop() {
stopMotors();
Serial.println("🛑 EMERGENCY STOP!");
while(1) {
// Halt forever
delay(100);
}
}
void changeState(RobotState newState) {
currentState = newState;
stateStartTime = millis();
delay(150);  // State transition delay
Serial.print("State changed to: ");
switch(newState) {
case SEARCHING_CUBE: Serial.println("SEARCHING_CUBE"); break;
case APPROACHING_CUBE: Serial.println("APPROACHING_CUBE"); break;
case ALIGNING_TO_CUBE: Serial.println("ALIGNING_TO_CUBE"); break;
case GRIPPING_CUBE: Serial.println("GRIPPING_CUBE"); break;
case SEARCHING_COLOR: Serial.println("SEARCHING_COLOR"); break;
case PLACING_CUBE: Serial.println("PLACING_CUBE"); break;
}
}
void checkBatteryVoltage() {
static unsigned long lastCheck = 0;
if (millis() - lastCheck > 10000) {  // Check every 10 seconds
int raw = analogRead(BATTERY_VOLTAGE);
float voltage = raw * (5.0 / 1023.0) * 3;  // Assuming voltage divider
if (voltage < 6.0) {  // Low battery threshold
  Serial.println("⚠️ Warning: Battery voltage low!");
}

lastCheck = millis();
}
}