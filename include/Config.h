#pragma once
// Use MPU6050 (definite)
#define USE_MPU6050

// Control loop
#define CONTROL_LOOP_HZ 200
#define CONTROL_LOOP_DT_S (1.0f / CONTROL_LOOP_HZ)

// PID safe limits (angular velocity output, deg/s)
#define PID_OUTPUT_MIN_F -1000.0f
#define PID_OUTPUT_MAX_F 1000.0f

// PID defaults (used only if no stored values exist)
#define DEFAULT_PID_KP 1.0f
#define DEFAULT_PID_KI 0.0f
#define DEFAULT_PID_KD 1.0f

// NVS namespace and keys
#define PREFS_NAMESPACE "sbr2"   // NVS namespace
#define PREFS_KEY_KP   "kp"
#define PREFS_KEY_KI   "ki"
#define PREFS_KEY_KD   "kd"


// Steps mapping (tune later)
#define STEPS_PER_DEGREE (3200.0f/360.0f) // ≈ 8.8888889

// I2C pins (from KiCad nets)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_CLOCK_HZ 100000UL

// Motor pins
#define PITCH_STEP_PIN 4
#define PITCH_DIR_PIN 0
#define ROLL_STEP_PIN 2
#define ROLL_DIR_PIN 15
#define PITCH_EN_PIN -1
#define ROLL_EN_PIN -1

// Motor sign configuration: adjust if a motor spins the opposite direction
#define LEFT_MOTOR_SIGN 1
#define RIGHT_MOTOR_SIGN -1

// Misc
#define SERIAL_BAUD 115200


