#include "Config.h" // for STEPS_PER_DEGREE etc.
#include "HardwareMap.h" // optional: provides PITCH_STEP, PITCH_DIR, etc.
#include "BotController.h"
#include <Preferences.h>


BotController::BotController() : leftMotor(PITCH_STEP, PITCH_DIR, PITCH_EN), rightMotor(ROLL_STEP, ROLL_DIR, ROLL_EN) {
  portMUX_INITIALIZE(&mux);
  pendingPid = false;
  stepsPerDegree = STEPS_PER_DEGREE;
}

void BotController::begin() {
  leftMotor.begin();
  rightMotor.begin();
  // init IMU with retry+recover on failure
  if (!imu.begin()) {
    Serial.println("IMU init failed — attempting I2C recover + retry");
    IMU::i2cBusRecover(I2C_SDA_PIN, I2C_SCL_PIN);
    delay(50);
    if (!imu.begin()) {
      Serial.println("IMU init failed after recover. Continuing without IMU.");
    } else {
      Serial.println("IMU init succeeded after recover.");
    }
  }
  ble.begin();
  // default PID values (Kp, Ki, Kd) in degrees/deg-s/seconds form
  loadStoredPid();
}

void BotController::update(float dt) {
  // read imu
  imu.update(dt);

  // check for pending BLE params and apply safely
  PIDParams p;
  if (ble.takePending(p)) {
    portENTER_CRITICAL(&mux);
    pendingParams = p;
    pendingPid = true;
    portEXIT_CRITICAL(&mux);
  }
  if (pendingPid) applyPendingPid();

  // compute control for pitch
  float currentPitch = imu.getPitch();

  // Non-blocking telemetry emit (throttled).
  static unsigned long _lastTelemetryMs = 0;
  const unsigned long _telemetryIntervalMs = 20; // 50 Hz
  unsigned long _nowMs = millis();
  if (_nowMs - _lastTelemetryMs >= _telemetryIntervalMs) {
    _lastTelemetryMs = _nowMs;
    Serial.printf("PITCH:%.2f ROLL:%.2f YAW:%.2f\n", currentPitch, imu.getRoll(), imu.getYaw());
  }

  // PID compute: returns angular velocity (deg/s)
  float pitchOutDegPerSec = pitchPid.compute(targetPitch, currentPitch, dt); // out in deg/s
  float pitchStepsPerSec = pitchOutDegPerSec * stepsPerDegree; // convert to steps/sec once

  // apply motor sign configuration so left/right can be inverted without code edits
  float leftSteps = pitchStepsPerSec * LEFT_MOTOR_SIGN;
  float rightSteps = pitchStepsPerSec * RIGHT_MOTOR_SIGN;

  // drive both wheels from pitch controller (non-blocking)
  leftMotor.setSpeedStepsPerSec(leftSteps);
  rightMotor.setSpeedStepsPerSec(rightSteps);

  // non-blocking stepper service (must be called frequently)
  leftMotor.runSpeed();
  rightMotor.runSpeed();
}

void BotController::requestPidParams(const PIDParams &p) {
  portENTER_CRITICAL(&mux);
  pendingParams = p;
  pendingPid = true;
  portEXIT_CRITICAL(&mux);
}

void BotController::applyPendingPid() {
  portENTER_CRITICAL(&mux);
  if (pendingPid) {
    // interpret pendingParams as continuous (Kp, Ki_per_s, Kd_seconds)
    pitchPid.setTunings(pendingParams.kp, pendingParams.ki, pendingParams.kd);
    pitchPid.reset();
    // persist immediately so values survive power cycles
    savePidToStorage(pendingParams.kp, pendingParams.ki, pendingParams.kd);
    pendingPid = false;
  }
  portEXIT_CRITICAL(&mux);
}

void BotController::printCurrentPid() {
  float kp, ki, kd;
  pitchPid.getTunings(kp, ki, kd);
  Serial.printf("KP: %.6f KI: %.6f KD: %.6f\n", kp, ki, kd);
}

void BotController::loadStoredPid() {
  Preferences prefs;
  // open namespace for read/write
  if (!prefs.begin(PREFS_NAMESPACE, false)) {
    Serial.println("Prefs begin failed - using defaults");
    // use compile-time defaults
    pitchPid.begin(DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
    return;
  }

  float kp = prefs.getFloat(PREFS_KEY_KP, DEFAULT_PID_KP);
  float ki = prefs.getFloat(PREFS_KEY_KI, DEFAULT_PID_KI);
  float kd = prefs.getFloat(PREFS_KEY_KD, DEFAULT_PID_KD);

  prefs.end();

  Serial.printf("Loaded PID from NVS: KP=%.6f KI=%.6f KD=%.6f\n", kp, ki, kd);
  pitchPid.begin(kp, ki, kd, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
}

void BotController::savePidToStorage(float kp, float ki, float kd) {
  Preferences prefs;
  if (!prefs.begin(PREFS_NAMESPACE, false)) {
    Serial.println("Prefs begin failed - cannot save PID");
    return;
  }
  prefs.putFloat(PREFS_KEY_KP, kp);
  prefs.putFloat(PREFS_KEY_KI, ki);
  prefs.putFloat(PREFS_KEY_KD, kd);
  prefs.end();
  Serial.printf("Saved PID to NVS: KP=%.6f KI=%.6f KD=%.6f\n", kp, ki, kd);
}
