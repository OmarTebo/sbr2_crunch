#include "Config.h"       // for STEPS_PER_DEGREE etc.
#include "HardwareMap.h"  // optional: provides PITCH_STEP, PITCH_DIR, etc.
#include "BotController.h"
#include <Arduino.h>

BotController::BotController()
: pitchMotor(PITCH_STEP, PITCH_DIR, PITCH_EN),
  rollMotor(ROLL_STEP, ROLL_DIR, ROLL_EN)
{
  portMUX_INITIALIZE(&mux);
  pendingPid = false;
  stepsPerDegree = STEPS_PER_DEGREE;
}

void BotController::begin() {
  pitchMotor.begin();
  rollMotor.begin();
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

  // default PID values (Kp, Ki, Kd)
  pitchPid.begin(1.0f, 0.0f, 0.01f, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);

  // keep motors disabled by default in hardware (enable pin behavior depends on wiring)
  // leave enable state to MotorDriver::begin() default
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

  // Non-blocking telemetry emit (throttled). Prints one-line: PITCH:<deg> ROLL:<deg> YAW:<deg>
  static unsigned long _lastTelemetryMs = 0;
  const unsigned long _telemetryIntervalMs = 20; // 50 Hz
  unsigned long _nowMs = millis();
  if (_nowMs - _lastTelemetryMs >= _telemetryIntervalMs) {
    _lastTelemetryMs = _nowMs;
    Serial.printf("PITCH:%.2f ROLL:%.2f YAW:%.2f\n", currentPitch, imu.getRoll(), imu.getYaw());
  }

  float pitchOut = pitchPid.compute(targetPitch, currentPitch, dt); // out in steps/sec
  float pitchSteps = pitchOut * stepsPerDegree;

  // drive both wheels from pitch controller. invert one side if wiring needs it.
  pitchMotor.setSpeedStepsPerSec(pitchSteps);
  rollMotor.setSpeedStepsPerSec(pitchSteps);

  // non-blocking stepper service
  pitchMotor.runSpeed();
  rollMotor.runSpeed();
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
    pendingPid = false;
  }
  portEXIT_CRITICAL(&mux);
}

void BotController::printCurrentPid() {
  float kp, ki, kd;
  pitchPid.getTunings(kp, ki, kd);
  Serial.printf("KP: %.6f KI: %.6f KD: %.6f\n", kp, ki, kd);
}
