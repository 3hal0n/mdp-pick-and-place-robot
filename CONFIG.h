#pragma once

// CONFIG.h - Consolidated pin and constant definitions for Final_Full_Code.ino
// Copy this into the Arduino project and include with: #include "CONFIG.h"

#include <Arduino.h>

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
#define STEPS_PER_REV 200
#define WHEEL_DIAMETER 6.5      // cm - measure your wheel
#define WHEEL_BASE 20.0         // cm - distance between wheels
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
#define KP_WALL 0.8    // Proportional gain
#define KI_WALL 0.1    // Integral gain
#define KD_WALL 0.3    // Derivative gain

// ============ SENSOR FILTERING ============
#define FILTER_SIZE 5

// Misc
#define MAX_PLACED_CUBES 5
#define PATH_MEMORY_SIZE 100

// Helpful notes:
// - Update WHEEL_DIAMETER and WHEEL_BASE to match your robot for accurate odometry.
// - Tune the PID (KP_WALL, KI_WALL, KD_WALL) on the surface where the robot will run.
// - Use separate motor supply for steppers and common ground.
