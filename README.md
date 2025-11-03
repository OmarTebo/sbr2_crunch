# Self-Balancing Robot (SBR) — README

**Purpose:** Firmware for a two-wheeled self-balancing robot running on an ESP32 (ESP-WROOM-32). This README gives new developers the quick start steps, important callouts, and guidance for migrating to a discrete/Tustin PID if desired.

---

## Quick start (TL;DR)

### Prerequisites
- PlatformIO (use the `platformio.ini` environments).
- VS Code recommended with PlatformIO extension.
- Python 3 (for tools/pid_gui.py).
- USB cable and bench power supply for motors. Keep motors disabled while wiring.

### Build & upload
```bash
# build and upload (PlatformIO CLI)
pio run -e <env> -t upload

# open serial monitor (project uses 115200 by default)
pio device monitor -b 115200
```
Replace `<env>` with the environment name in `platformio.ini` (the board variant you use).

### Minimal wiring sanity check
- ESP32 power: VIN/5V and GND (verify regulator wiring before connecting motor power).
- IMU (MPU6050): SDA, SCL, VCC, GND. Use pull-ups and keep wiring short.
- Motor drivers: wire STEP/DIR/ENABLE according to `include/HardwareMap.h`. Keep VMOT disconnected while verifying logic.

### Run & monitor
- Upload firmware, open serial monitor, watch boot logs for IMU init and PID defaults.
- Use `SET PID` / `GET PID` via serial (or BLE) for live tuning.
- `tools/pid_gui.py` can visualize telemetry and change gains — use serial for bulk telemetry.

### Single-source-of-truth
- `include/Config.h` holds key constants: `CONTROL_LOOP_HZ`, `STEPS_PER_DEGREE`, default PID gains, and hardware pin macros.

---

## Callouts (important — read before editing)

### Units & PID semantics (VERY IMPORTANT)
- **Ki is expressed per second** (`Ki_per_s`).  
- **Kd is expressed as a time constant in seconds** (`Kd_seconds`).  
- `PIDController::compute()` expects `dt` in seconds. Always record and communicate the units when sharing gains.

### Historical fixed-point / per-sample gotcha
- Older branches used fixed-point scaling (scale = **4096**) and per-sample gains. **Do not mix** per-sample and per-second gains — doing so results in incorrect effective gain and possible integer overflow if fixed-point math is reintroduced.

### Do NOT replace the IMU library
- The project uses a custom MPU6050 library in `lib/MPU6050/` (Kalman/fusion + preprocessing + I²C recovery). This implementation includes behavior and recovery logic that other libraries do not. **Do not replace** it with Adafruit or other off-the-shelf IMU libraries without careful validation.

### STEPS_PER_DEGREE and microstepping
- `STEPS_PER_DEGREE` is defined in `include/Config.h`. If you change microstepping, gear ratio, or pulleys, update this value and run bench tests to verify mapping.

### I²C robustness & recovery
- Symptom: IMU stops responding or I²C appears hung. Quick recovery steps:
  1. Verify wiring and pull-ups.
  2. Trigger the software I²C reinit path (the MPU driver logs reinit attempts at boot or on error).
  3. Power-cycle the IMU if software recovery fails.
- Use strong pull-ups and keep SDA/SCL traces short.

### Motor wiring & safety
- Check `include/HardwareMap.h` for pin mapping and `LEFT_MOTOR_SIGN` / `RIGHT_MOTOR_SIGN` macros (invert if motors spin the wrong way).
- **Emergency stop:** the ENABLE pin can be used to disable drivers. Keep a hardware kill switch on the bench as a safety best practice.

### Serial & BLE tuning interfaces
- Serial CLI supports `SET PID`, `GET PID`, `HELP`. Use serial for verbose telemetry and `tools/pid_gui.py`.
- BLE mirrors PID writes for remote tuning but is less convenient for logs.

### Persistence & config
- PID defaults and last-saved gains persist to NVS via `BotController`. To reset, clear NVS or use the provided reset command in Serial CLI (see code for exact command).

### Short debugging checklist
- Robot leans / oscillates: verify `STEPS_PER_DEGREE`, motor signs, and `CONTROL_LOOP_HZ`.
- IMU data stuck/hangs: follow I²C recovery steps.
- Motors jitter: check microstepping, `AccelStepper` configuration, and ensure no blocking delays in the main loop.

### Recommended immediate TODOs (for repo & README)
- Add explicit `EMERGENCY_STOP` command and hardware kill-switch guidance.
- Add a bench test to measure/verify `STEPS_PER_DEGREE` (pulse test + recorded degrees).
- Add clear examples of PID units in Serial/BLE examples.
- Add a deterministic control-task example using a hardware timer or pinned RTOS task.

---

## Discrete PID (Tustin) — migration guide & TODOs

> Short verdict: migrate to a discrete/Tustin PID **only** if you can enforce a stable, low-jitter sample period (e.g., run the PID on a hardware timer or a pinned RTOS task at 200 Hz). Otherwise, keep the existing dt-aware PID or implement an AUTO hybrid.

### Why consider discrete / Tustin
- **CPU efficiency:** per-tick cost is minimal (few multiplies + adds). Good for ESP32 running IMU, motor drivers, and BT.
- **Frequency-domain fidelity:** bilinear/Tustin mapping preserves the continuous PID frequency response better than naive Euler conversions when sample time is stable.
- **Deterministic behavior:** coefficients precomputed for a fixed T yield repeatable controller dynamics.

### Why you might not migrate
- **Requires a stable sample period.** Jitter invalidates precomputed coefficients and degrades stability.
- **Anti-windup complexity.** Difference-equation forms can hide the explicit integral state; ensure integrator clamping or back-calculation is provided.
- **Tooling & unit mismatches.** Keep `Ki_per_s` and `Kd_seconds` semantics and provide conversion adapters — mixing conventions causes bugs.

### Migration checklist (practical)
1. Add a `PIDInterface` with methods used by `BotController`: `setTunings(kp,ki,kd)`, `setSampleTime_ms(T_ms)`, `compute(setpoint, measurement)`, `setOutputLimits(min,max)`, `reset()`.
2. Implement `PID_Tustin` that accepts continuous gains `(kp, ki_per_s, kd_seconds)` and computes the Tustin coefficients using the configured `T`.
3. Enforce a deterministic control task (hardware timer / RTOS task) and measure jitter (mean, max, stddev). Aim for jitter < ±5% of `T`.
4. Add runtime mode `MODE = {DT_AWARE, DISCRETE_TUSTIN, AUTO}`. In AUTO, switch conservatively based on jitter history.
5. Preserve anti-windup (clamping or back-calculation) and ensure consistent behavior across mode switches.
6. Add telemetry/CLI hooks to dump coefficients (b0..b2, a1..a2) and dt stats.
7. Create bench tests: step response logs for both controllers using identical continuous gains (CSV output).
8. Document conversion example: continuous gains → T → discrete coefficients for a chosen `CONTROL_LOOP_HZ` (e.g., 200 Hz).

### Debug & acceptance tests
- Dump coefficients at startup and assert finiteness and expected magnitudes.
- Run side-by-side step response tests (DT-aware vs DISCRETE) and compare rise time, overshoot, RMSE over 5 s.
- Monitor loop `dt` across a sliding window and flag instability if `max - mean > 0.05*T`.

### Prioritized TODOs (copy-ready)
**Short-term**
- [ ] Add `PIDInterface` and `PID_Tustin` skeleton.
- [ ] Add CLI dump for coefficients and dt stats.
- [ ] Add bench test comparing DT-aware vs DISCRETE.

**Mid-term**
- [ ] Implement shared anti-windup (integrator clamping / back-calculation).
- [ ] Add runtime AUTO mode with hysteresis for switching.
- [ ] Add deterministic control-task example.

**Long-term**
- [ ] Add unit tests and CI regression comparisons.
- [ ] Add recorded telemetry dataset and a Python replay tool.

---

## Project map (brief file roles)
- `src/main.cpp` — fixed-timestep loop + startup.
- `src/BotController.cpp` + `include/BotController.h` — top-level logic, NVS persistence.
- `src/IMU.cpp` + `lib/MPU6050/` — custom MPU6050 driver, fusion, Kalman, I²C recovery (**do not replace**).
- `src/PIDController.cpp` + `include/PIDController.h` — current dt-aware PID implementation (units: Ki_per_s, Kd_seconds).
- `src/MotorDriver.cpp` + `include/MotorDriver.h` — AccelStepper wrapper; sets steps/sec.
- `src/SerialBridge.cpp` / `src/BLEHandler.cpp` — tuning interfaces.
- `tools/pid_gui.py` — host-side telemetry/tuning helper.
- `include/Config.h`, `include/HardwareMap.h` — pins, `STEPS_PER_DEGREE`, `CONTROL_LOOP_HZ`.

---

## Troubleshooting (short)
- IMU hang: check wiring, trigger software reinit (driver logs), power-cycle sensor.
- Oscillation: check `STEPS_PER_DEGREE`, motor signs, loop frequency, and PID units.
- Motor jitter: verify microstepping, AccelStepper settings, and that main loop is non-blocking.

---

## How to contribute
- Keep changes small and focused. Preferred workflow:
  1. Fork → feature branch → PR against `main`.
  2. Include a short description and test steps in the PR.
  3. If changing IMU behavior or PID semantics, include bench logs that demonstrate effect.

---

## Short TODO checklist (top of README)
- [ ] Add EMERGENCY_STOP command & docs.
- [ ] Add `PID_Tustin` module + `PIDInterface`.
- [ ] Add deterministic control-task example (timer).
- [ ] Add `STEPS_PER_DEGREE` bench test script and results capture.
- [ ] Document IMU recovery steps and keep custom MPU6050 lib.

---

## Where to look next (quick links)
- `include/Config.h` — constants & defaults
- `include/HardwareMap.h` — pin mapping
- `lib/MPU6050/` — IMU implementation (do NOT replace)
- `src/PIDController.cpp` — current PID
- `src/BotController.cpp` — where PID is used
- `tools/pid_gui.py` — tuning/visualization

---

If you'd like, I can:
- write the `PIDInterface` + `PID_Tustin` skeleton files next, ready to compile, or  
- create the bench test script (`/tools/tests/`) that logs CSV telemetry for side-by-side comparison.

Which should I do next?
