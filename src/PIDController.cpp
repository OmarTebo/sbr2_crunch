#include "PIDController.h"
#include "Config.h"
#include <cmath>

PIDController::PIDController() {
  kp = ki = kd = 0.0f;
  integral = prevError = derivFiltered = 0.0f;
  d_alpha = 0.6f; // derivative filter
  outMin = PID_OUTPUT_MIN_F;
  outMax = PID_OUTPUT_MAX_F;
}

void PIDController::begin(float _kp, float _ki, float _kd, float outMin_f, float outMax_f) {
  kp = _kp;
  ki = _ki; // Ki_per_s (continuous)
  kd = _kd; // Kd_seconds (continuous)
  outMin = outMin_f;
  outMax = outMax_f;
  reset();
}

void PIDController::setTunings(float _kp, float _ki, float _kd) {
  kp = _kp;
  ki = _ki;
  kd = _kd;
}

void PIDController::setTuningsContinuous(float Kp, float Ki_per_s, float Kd_seconds, float sampleRateHz) {
  // keep continuous-time values; compute() is dt-aware
  kp = Kp;
  ki = Ki_per_s;
  kd = Kd_seconds;
}

float PIDController::compute(float setpoint, float measurement, float dt_s) {
  if (dt_s <= 0.0f) return 0.0f;
  float error = setpoint - measurement; // degrees

  // P term (deg * unitless kp) -> units: deg * kp (interpreted as deg/s contribution depending on kp units)
  float P = kp * error;

  // I term (continuous Ki_per_s): integral accumulates error * dt (deg * s)
  integral += error * dt_s;
  // anti-windup: prevent integral growing beyond what would saturate output
  float integralLimit = 0.0f;
  if (ki != 0.0f) {
    // out = ki * integral when I dominates; to keep magnitude bounded, limit integral to outMax/ki
    integralLimit = fabs(outMax / (ki != 0.0f ? ki : 1.0f));
  } else {
    integralLimit = 1e6f;
  }
  if (integral > integralLimit) integral = integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;
  float I = ki * integral; // units: (1/s) * (deg*s) => deg

  // D term: derivative of error (deg/s)
  float d_raw = (error - prevError) / dt_s;
  derivFiltered = d_alpha * d_raw + (1.0f - d_alpha) * derivFiltered;
  float D = kd * derivFiltered; // kd (seconds) * (deg/s) => deg

  prevError = error;

  // Sum terms. Because we want output to be angular velocity (deg/s), choose kp/ki/kd accordingly when tuning.
  float out = P + I + D;

  // clamp
  if (out > outMax) out = outMax;
  if (out < outMin) out = outMin;
  return out; // deg/s
}

void PIDController::reset() {
  integral = 0.0f;
  prevError = 0.0f;
  derivFiltered = 0.0f;
}

void PIDController::getTunings(float &out_kp, float &out_ki, float &out_kd) {
  out_kp = kp;
  out_ki = ki;
  out_kd = kd;
}


