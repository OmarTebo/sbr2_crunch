# SBR — Detailed Documentation (classes, functions, roles)

This file documents the public API in `include/` and explains the runtime behaviour implemented in `src/`. It focuses on headers and implementations in `include/` and `src/` and explains roles, units, and usage examples.

---

## Table of contents

* Config.h
* Types.h
* HardwareMap.h
* IMU (IMU.h / IMU.cpp)
* PIDController (PIDController.h / PIDController.cpp)
* MotorDriver (MotorDriver.h / MotorDriver.cpp)
* BotController (BotController.h / BotController.cpp)
* SerialBridge (SerialBridge.h / SerialBridge.cpp)
* BLEHandler (BLEHandler.h / BLEHandler.cpp)
* main.cpp — program flow
* lib/MPU6050 — notes
* Tuning examples & snippets

---

## Config.h (constants & pin mapping)

Purpose: a single place for compile-time constants and pin mappings.

Key constants and their roles:

* CONTROL_LOOP_HZ — control loop frequency (typical default 200).
* STEPS_PER_DEGREE — conversion factor from degrees to motor steps (e.g., 3200.0/360.0).
* I2C pins: I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ.
* Motor pins: PITCH_STEP_PIN, PITCH_DIR_PIN, ROLL_STEP_PIN, ROLL_DIR_PIN, and optional EN pins.
* SERIAL_BAUD — serial monitor baud rate (e.g., 115200).
* Default PID values (KP, KI, KD) and limits for outputs.

Usage: change values here before flashing if your wiring or mechanical gearing differs.

---

## Types.h

Defines small shared structures and simple types used across modules.

Example:

* struct PIDParams { float kp; float ki; float kd; };

Role: used to transfer PID sets between BLE, serial and the controller and to store/restore tunings.

---

## HardwareMap.h

Purpose: thin mapping of config macros to named constants used in constructors and object initialization.

Role: keeps code readable by centralizing pin names (e.g., PITCH_STEP → PITCH_STEP_PIN).

---

## IMU — IMU.h / IMU.cpp

Public interface summary

* Class: IMU
* Public methods:

  * IMU() — constructor.
  * bool begin() — initialize sensor, fusion, and perform startup calibration.
  * void update(float dt) — update internal fusion; dt is seconds.
  * float getPitch() — returns pitch in degrees.
  * float getRoll() — returns roll in degrees.
  * float getYaw() — yaw in degrees.
  * unsigned long lastUpdateMillis() — timestamp of last successful update.
  * static void i2cBusRecover(int sdaPin, int sclPin) — attempt to recover stuck I2C bus.

Roles & details

* The IMU wrapper uses a bundled MPU6050 driver and a sensor fusion module. On begin() it initializes the device and computes startup zero offsets by averaging several samples.
* update(dt) calls into the underlying driver/fusion, applies offsets, and stores filtered angles. If reads stall, the wrapper can attempt I2C bus recovery and re-init.
* i2cBusRecover toggles SCL lines to try to free a stuck SDA and re-initializes the Wire/I2C interface.
* Units: angles are in degrees; dt is in seconds.

Example usage (indented code block, do not use fenced code delimiters)
IMU imu;
if (!imu.begin()) {
IMU::i2cBusRecover(I2C_SDA_PIN, I2C_SCL_PIN);
imu.begin();
}
imu.update(0.005f);
float pitch = imu.getPitch();

---

## PIDController — PIDController.h / PIDController.cpp

Public API summary

* Class: PIDController
* Methods:

  * PIDController()
  * void begin(float kp, float ki, float kd, float outMin, float outMax)
  * void setTunings(float kp, float ki, float kd)
  * void setTuningsContinuous(float Kp, float Ki_per_s, float Kd_seconds, float sampleRateHz)
  * float compute(float setpoint, float measurement, float dt_s) — returns angular velocity (deg/s)
  * void reset()
  * void getTunings(float &out_kp, float &out_ki, float &out_kd)

Semantics & units

* kp: proportional gain (unitless multiplier on degree error).
* ki: Ki_per_s (integral gain expressed per second). The integral accumulates error*dt; so ki * integral contributes in the same units as the proportional term.
* kd: Kd_seconds (derivative time constant in seconds). The derivative is multiplied by kd to give a term aligned with other gains.
* compute() returns an angular velocity in deg/s. The controller expects setpoint and measurement in degrees, and dt in seconds.

Implementation notes

* Anti-windup via clamping integral term.
* A simple derivative filter (alpha or similar) smooths the derivative term.
* The API also supports setting tunings expressed as continuous-time parameters (Ki per second, Kd in seconds) which is convenient when changing sample rate.

Example
PIDController pid;
pid.begin(1.0f, 0.0f, 0.01f, -1000.0f, 1000.0f);
float out_deg_per_s = pid.compute(target_deg, measured_deg, 0.005f);

---

## MotorDriver — MotorDriver.h / MotorDriver.cpp

Public API summary

* Class: MotorDriver
* Methods:

  * MotorDriver(uint8_t stepPin, uint8_t dirPin, int8_t enPin = -1)
  * void begin()
  * void setSpeedStepsPerSec(float stepsPerSec)
  * void runSpeed()  — must be called frequently (non-blocking)
  * void enable(bool en)
  * void setMaxSpeed(float s)
  * void setAcceleration(float a)

Roles & behaviour

* Wraps AccelStepper (or equivalent) in DRIVER mode to generate step pulses for A4988-style drivers.
* setSpeedStepsPerSec expects steps per second; runSpeed should be called often (e.g., each control tick).
* enable toggles the EN pin; board wiring convention (active-low or active-high) must be observed.
* The driver code handles conversion of angular velocity to step rates elsewhere (BotController).

Example usage
MotorDriver left(PITCH_STEP, PITCH_DIR, PITCH_EN);
left.begin();
left.setSpeedStepsPerSec(100.0f); // 100 steps/sec
left.runSpeed(); // call each tick

---

## BotController — BotController.h / BotController.cpp

Purpose

High-level orchestrator that ties IMU, PID controllers, motor drivers, and communication together. It handles safety, parameter updates, persistence, telemetry, and the core control law.

Public API (summary)

* Class: BotController
* Methods:

  * BotController()
  * void begin()
  * void update(float dt)  — call every control tick; dt is seconds
  * void requestPidParams(const PIDParams &p)
  * void printCurrentPid()
* Public members:

  * MotorDriver leftMotor;
  * MotorDriver rightMotor;
  * float targetPitch;
  * float targetRoll;

Responsibilities & flow

* begin(): initialize motors, attempt IMU init (with I2C recover on failure), initialize BLE and serial, and load stored PID from NVS/Preferences.
* update(dt): primary steps:

  1. imu.update(dt) to refresh angles.
  2. Apply pending PID params from BLE/Serial atomically and persist them.
  3. Compute pitch PID: pitchPid.compute(targetPitch, currentPitch, dt) → deg/s.
  4. Convert deg/s to steps/sec using STEPS_PER_DEGREE and apply motor signs.
  5. Set motor speeds and call runSpeed on each motor.
* Telemetry: the controller prints formatted telemetry at a throttled rate, typically lower than the control frequency to avoid timing disruption.

Notes

* The conversion factor and motor sign (left/right inversion) are applied here to produce correct wheel motions.
* Safety: consider automatically disabling motors when IMU health is poor.

---

## SerialBridge — SerialBridge.h / SerialBridge.cpp

Public API summary

* Class: SerialBridge
* Methods:

  * SerialBridge()
  * void begin(unsigned long baud)
  * bool poll(PIDParams &paramsOut) — parses lines and returns true if SET PID was received
  * void printHelp()
  * void printCurrent(PIDController &pid)
  * bool consumeGetPidRequest()

Behavior

* A simple line parser that accepts commands such as:

  * SET PID <kp> <ki> <kd>
  * GET PID
  * HELP
* poll() reads available serial data, parses commands, and if a SET PID command is parsed returns parsed PIDParams via paramsOut.
* GET PID is implemented as a request flag; consumeGetPidRequest returns and clears it so main or controller can print safely from the correct context.
* printCurrent() reads PID tunings from a PIDController instance and prints them.

Example serial commands
SET PID 1.000 0.000 0.010
GET PID
HELP

---

## BLEHandler — BLEHandler.h / BLEHandler.cpp

Public API summary

* Class: BLEHandler
* Methods:

  * BLEHandler()
  * void begin()
  * bool takePending(PIDParams &out) — atomically consumes pending BLE writes and returns true if new values were provided

Behavior

* Exposes BLE characteristics for KP, KI, KD (and possibly a control/telemetry characteristic).
* On characteristic writes, stores the values in a pending structure and sets an atomic flag.
* takePending() is used by BotController to safely consume pending parameter updates and apply them inside the main control context.

Notes

* BLE operations are handled in the BLE task/context; only the takePending method touches controller state to avoid race conditions.

---

## main.cpp — program entry & fixed-timestep loop

Program flow summary

* Constructs the main components: BotController, SerialBridge, BLEHandler.
* Initializes peripherals and services in setup/begin calls.
* Implements an accumulator-based fixed timestep loop:

  * Tracks elapsed time and accumulates into a variable.
  * While accumulator >= tickDuration:

    * call controller.update(dt) with dt in seconds (tickDuration).
    * accumulator -= tickDuration.
  * Outside the fixed tick loop it polls serial and BLE for incoming commands and handles non-time-critical housekeeping.
* This structure produces stable fixed-dt control while allowing slower tasks to run when CPU time remains.

Timing conventions

* dt is in seconds (e.g., 0.005 for 200 Hz).
* Control computations use dt consistently across IMU update, PID compute, and motor speed updates.

---

## lib/MPU6050 — notes

* A bundled MPU6050 driver, fusion, and Kalman filter code are present under lib/MPU6050/.
* The IMU wrapper delegates to this library for sensor reads and sensor fusion. Startup calibration typically averages many samples using the MPU API to derive static offsets.
* For filter internals and tuning, inspect MPU6050_Fusion and Kalman files in the lib folder.

---

## Examples & snippets

Converting PID output (deg/s) to stepper speeds (steps/sec)
float pitchOutDegPerSec = pitchPid.compute(targetPitch, currentPitch, dt);
float pitchStepsPerSec = pitchOutDegPerSec * STEPS_PER_DEGREE;
leftMotor.setSpeedStepsPerSec(pitchStepsPerSec * LEFT_MOTOR_SIGN);
rightMotor.setSpeedStepsPerSec(pitchStepsPerSec * RIGHT_MOTOR_SIGN);

Serial command example
SET PID 1.000 0.000 0.010
GET PID
HELP

IMU init and update example
IMU imu;
if (!imu.begin()) {
IMU::i2cBusRecover(I2C_SDA_PIN, I2C_SCL_PIN);
imu.begin();
}
imu.update(0.005f);
float pitch = imu.getPitch();

---

## Implementation notes & recommendations

* Units: everywhere the code expects degrees and deg/s for angle and rate. Keep those units consistent across IMU, PID and motor conversions.
* Safety: IMU health (stalled reads) triggers I2C bus recovery and reinit in IMU::update. Add an explicit motor-disable path when IMU is unhealthy for added safety.
* Persistence: PID tuning is stored in NVS/Preferences. BotController contains load and save helpers for persistent settings.
* Telemetry: throttle prints so they do not break timing; prefer lower telemetry frequency (e.g., ~50 Hz) than control tick rate.

---

## What was documented, assumptions & caveats

* This documentation focuses on the headers and the `src/` implementations in the repository (IMU, PIDController, MotorDriver, BotController, SerialBridge, BLEHandler and main.cpp).
* Descriptions are based on function names, comments, and typical embedded-control conventions present in the source.
* For full algorithmic details (sensor fusion, Kalman internals), consult the files in lib/MPU6050/.

---

## Recommended next steps

* Add Doxygen-style comments directly in headers for automated API generation.
* Add a README in lib/MPU6050/ summarizing the fusion algorithm and any tuning knobs.
* Add a build flag or runtime mode to run without enabling motors for safe bench tuning and telemetry-only experiments.
