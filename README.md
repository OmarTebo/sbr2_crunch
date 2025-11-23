# Self-Balancing Robot (SBR)

Small, lightweight firmware for a two-wheeled self-balancing robot running on an ESP32,
using an MPU6050 IMU and A4988-driven stepper motors. The firmware implements a sensor
wrapper, a PID controller, and a stepper motor driver interface and exposes simple
serial and BLE interfaces for telemetry and PID tuning.

---

## Features

- IMU wrapper around a bundled MPU6050 library with startup calibration and I²C bus recovery.
- PID controller implemented with continuous-time `Ki_per_s` and `Kd_seconds` semantics. 
- Motor driver using `AccelStepper` to generate step pulses (non-blocking).
- BLE and Serial interfaces for remote telemetry and live PID changes.  
- Fixed-timestep control loop (accumulator/catch-up style) at `CONTROL_LOOP_HZ`.

---

## Quick hardware summary

- MCU: ESP32 (NodeMCU-32S / `esp32dev` PlatformIO board recommended).
- IMU: MPU6050 via I²C (SDA/SCL pins configurable in `Config.h`).
- Motors: Two NEMA steppers driven by A4988 drivers. Use microstepping and set current limits on the driver.
- Power: Separate motor VMOT and logic 3.3V (common ground).

Default pins and constants are in `include/Config.h`.

---

## Quick start (build & run)

Prereqs: PlatformIO (VSCode or CLI), ESP32 USB driver.

1. `git clone https://github.com/OmarTebo/sbr.git`
2. Open in VSCode (PlatformIO) or use PlatformIO CLI.
3. Configure any board/pins in `include/Config.h` if needed.
4. Build and upload using PlatformIO.
5. Open serial monitor at `SERIAL_BAUD` (default `115200`).
6. Perform IMU calibration (firmware does a startup calibration; you can re-run via serial commands). 

**Important safety notes**: Keep wheels off the ground during initial tuning and set A4988 current limits before enabling motors.

---

## Files & structure (high level)

- `include/` – public headers (IMU, PIDController, MotorDriver, BotController, SerialBridge, BLEHandler, Config). 
- `src/` – implementations: `main.cpp`, module `.cpp` files.
- `lib/MPU6050/` – bundled MPU6050 driver + fusion/kalman code used by the IMU wrapper.  
- `tools/pid_gui.py` – PC GUI utility for live PID tuning (serial).  
- `platformio.ini` – PlatformIO board/flags.

---

## Key runtime behavior

- The `main` loop runs a catch-up fixed-timestep loop and calls into `BotController::update(dt)`. `dt` uses seconds (e.g., 0.005s for 200Hz).
- IMU returns angles in **degrees** and rates in **degrees/second**. Use those units when tuning PID and mapping to steps.
- PID compute returns an **angular velocity** (deg/s). The controller converts that to steps/sec using `STEPS_PER_DEGREE`.

---

## PID tuning notes

- Use `Ki = 0` initially. Increase `Kp` until robot responds. Add `Kd` to damp oscillations. `Ki` only for steady-state correction. Example starting gains in code defaults: `Kp = 1.0`, `Ki = 0.0`, `Kd = 1.0` (defaults in `Config.h`).

---

## Telemetry & control

- SerialBridge supports `SET PID <kp> <ki> <kd>` and `GET PID`.  
- BLE exposes three characteristics for KP/KI/KD, allowing remote writes that are applied safely in the control loop.

---

## Contributing

- Keep public headers small and document units (degrees, deg/s, dt seconds).
- Put algorithm-heavy code in `.cpp` and keep `#include` minimal.
- Add unit tests for PID math and any filter code in `lib/MPU6050`.

---

## References / source files used

- `include/Config.h`, `include/IMU.h`, `include/PIDController.h`, `include/MotorDriver.h`, `include/BotController.h`, `include/SerialBridge.h` and their implementations in `src/`. (See repository for full sources).
