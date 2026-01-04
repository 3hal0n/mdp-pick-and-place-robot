# ME2610 Pick-and-Place Robot — Enhanced

**Overview**
- **Sketch:** [Final_Full_Code.ino](Final_Full_Code.ino)
- **Purpose:** Autonomous wall-following pick-and-place robot with radar sweep cube detection, multi-pass alignment, PID wall-following, gripper with retry logic, color detection, odometry, and path memory.
- **Highlights:** PID wall following, ultrasonic + IR filtering, multi-step alignment, battery monitoring, and simple odometry.

**Features**
- Wall-following via PID (`KP_WALL`, `KI_WALL`, `KD_WALL`).
- Radar sweep to detect cubes using two ultrasonic sensors (upper/lower) and height-difference detection.
- Multi-pass (coarse → fine) alignment before gripping.
- Gripper with retry and verification using a center ultrasonic sensor.
- Color detection (TCS3200) to place cubes on matching colored cells.
- Path memory and simple odometry to avoid revisiting locations.
- Emergency stop and battery voltage monitoring.

**Hardware / Pins (as used in the sketch)**
- **Steppers (A4988 drivers):** MOTOR_LEFT_STEP: 2, MOTOR_LEFT_DIR: 5, MOTOR_RIGHT_STEP: 3, MOTOR_RIGHT_DIR: 6
- **Ultrasonic Sensors:**
  - Left: TRIG 22, ECHO 23
  - Right: TRIG 24, ECHO 25
  - Radar Upper: TRIG 26, ECHO 27
  - Radar Lower: TRIG 28, ECHO 29
  - Gripper Center: TRIG 30, ECHO 31
- **IR Rear:** `A0`
- **Servos:** SERVO_RADAR 6, SERVO_GRIPPER 7, SERVO_TILT 8
- **TCS3200 Color Sensor:** S0=9, S1=10, S2=11, S3=12, OUT=13
- **Buttons:** START=14, STOP=15
- **Battery Sense:** `A1`

**Key Parameters (calibrate for your robot)**
- `STEPS_PER_REV`, `WHEEL_DIAMETER`, `WHEEL_BASE`, `STEPS_PER_CM` — set accurate wheel and stepper specs.
- `WALL_FOLLOW_DISTANCE` — desired distance from the left wall (cm).
- `GRIP_DISTANCE` — distance to maintain for gripping (cm).
- PID gains: `KP_WALL`, `KI_WALL`, `KD_WALL` — tune for smooth following.
- `CUBE_HEIGHT_DIFF` — threshold for detecting a cube from radar pair.

**Calibration & Tuning Notes**
- Measure `WHEEL_DIAMETER` and `WHEEL_BASE` precisely and recompute `STEPS_PER_CM`.
- Tune PID on a flat surface: start with `KI_WALL = 0`, increase `KP_WALL` until oscillation, then add small `KD_WALL` for damping, then slowly raise `KI_WALL` if steady-state error remains.
- Verify ultrasonic timing and sensor mounting; use `getUSDistanceMedian()` for critical reads.
- Adjust `COARSE_TOLERANCE` / `FINE_TOLERANCE` based on placement accuracy.

**How to Use**
- Open [Final_Full_Code.ino](Final_Full_Code.ino) in the Arduino IDE.
- Verify the board, port, and libraries: `AccelStepper`, `MultiStepper`, `Servo`.
- Upload sketch.
- Power robot. Press the **START** button (pin 14 pulled low in sketch) to begin; **STOP** button (pin 15) triggers emergency stop.
- Monitor `Serial` at 115200 baud for state logs and diagnostics.

**Behavior / State Machine**
- `SEARCHING_CUBE`: Sweeps radar and follows wall with PID when no cube found.
- `APPROACHING_CUBE`: Turns toward and approaches the detected cube.
- `ALIGNING_TO_CUBE`: Multi-pass alignment (coarse then fine) using radar angles.
- `GRIPPING_CUBE`: Attempts gripping with retries and verification.
- `SEARCHING_COLOR`: After grip, navigates to find a matching colored cell.
- `PLACING_CUBE`: Places cube, marks memory, backs up, and resumes search.

**Troubleshooting**
- Robot drifts or odometry incorrect: verify wheel diameter, step counts, and reduce slip by slower acceleration.
- Oscillatory wall following: reduce `KP_WALL` or increase `KD_WALL`.
- False cube detections: check radar sensor alignment and `CUBE_HEIGHT_DIFF` threshold.
- Color sensor unreliable: ensure consistent lighting and adjust detection thresholds in `detectColorWithFiltering()`.
- Motors not moving: confirm A4988 wiring and power supply (separate motor supply recommended).

**Files**
- Main sketch: [Final_Full_Code.ino](Final_Full_Code.ino)

**Next Steps / Improvements**
- Add configuration header for pin/constant overrides.
- Add unit-test harness (simulated sensors) or logging to file/SD for analysis.
- Add CW/CCW stepper microstepping configuration notes and motor current limiting steps.

---
**Wiring Diagram**

Below is a concise wiring table and a simple ASCII diagram showing how the main components map to pins used in the sketch.

Wiring table (component → board pin):
- **Left stepper STEP**: 2
- **Left stepper DIR**: 5
- **Right stepper STEP**: 3
- **Right stepper DIR**: 6
- **US Left TRIG / ECHO**: 22 / 23
- **US Right TRIG / ECHO**: 24 / 25
- **Radar upper TRIG / ECHO**: 26 / 27
- **Radar lower TRIG / ECHO**: 28 / 29
- **Gripper center TRIG / ECHO**: 30 / 31
- **IR rear**: A0
- **Battery sense**: A1
- **Servo (radar)**: 6
- **Servo (gripper)**: 7
- **Servo (tilt)**: 8
- **TCS3200 S0,S1,S2,S3,OUT**: 9,10,11,12,13
- **START button**: 14 (INPUT_PULLUP)
- **STOP button**: 15 (INPUT_PULLUP)

Simple ASCII wiring diagram (logical view):

  [Arduino Mega / UNO-style header]
  +---------------------------------------------+
  | 2 STEP_L    3 STEP_R    5 DIR_L    6 DIR_R |
  | 9..13 TCS3200  6 SERVO_RADAR 7 SERVO_GRIP  |
  | 8 SERVO_TILT  A0 IR_REAR  A1 BATTERY       |
  | 22/23 US_L   24/25 US_R   26/27 RAD_UPPER  |
  | 28/29 RAD_LO 30/31 US_CENTER 14 START 15 STOP|
  +---------------------------------------------+

Notes and wiring tips:
- Use the A4988 (or similar) with correct VMOT and GND wiring. Motor supply should be separate from the logic 5V supply; share common ground.
- Connect servo power to a stable 5V regulator if multiple servos draw current; avoid powering servos from the Arduino 5V regulator when using motors.
- Ultrasonic sensors: TRIG pins set as OUTPUT, ECHO pins to Arduino INPUT; ensure signal voltage levels are compatible (use level shifting if necessary).
- TCS3200: S0/S1 configure frequency scaling (the sketch sets S0 HIGH, S1 LOW for 20% scaling). Use the `COLOR_OUT` pin for pulse measurement.
- Buttons are pulled up in software (`INPUT_PULLUP`) so wire them to GND when pressed.

CONFIG.h
- A `CONFIG.h` file with all pin and constant definitions has been added to the project for easier configuration: `CONFIG.h`.
