#pragma once

// Use MPU6050 (definite)
#define USE_MPU6050
// #define USE_BNO055  // NEVER ENABLE — reserved but disabled

// Control loop
#define CONTROL_LOOP_HZ 200
#define CONTROL_LOOP_DT_S (1.0f / CONTROL_LOOP_HZ)

// PID safe limits (steps/sec)
#define PID_OUTPUT_MIN_F -1000.0f
#define PID_OUTPUT_MAX_F  1000.0f

// Steps mapping (tune later)
#define STEPS_PER_DEGREE (3200.0f/360.0f) // ≈ 8.8888889

// I2C pins (from KiCad nets)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_CLOCK_HZ 100000UL

// Motor pins (match nets.xml we parsed)
#define PITCH_STEP_PIN 4
#define PITCH_DIR_PIN  0
#define ROLL_STEP_PIN  2
#define ROLL_DIR_PIN   15
#define PITCH_EN_PIN -1
#define ROLL_EN_PIN  -1

// Misc
#define SERIAL_BAUD 115200
