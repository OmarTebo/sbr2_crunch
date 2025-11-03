#pragma once
#include <stdint.h>
#include "Config.h" // gives PID_OUTPUT_MIN_F and PID_OUTPUT_MAX_F

// compute: setpoint and measurement in same units (degrees).
// returns control in angular velocity (deg/s) (float).
float compute(float setpoint, float measurement, float dt_s);

class PIDController {
public:
  PIDController();
  void begin(float kp=0.5f, float ki=0.0f, float kd=0.0f, float outMin=PID_OUTPUT_MIN_F, float outMax=PID_OUTPUT_MAX_F);
  void setTunings(float kp, float ki, float kd);
  // Convenience: set using continuous-time Ki_per_s and Kd_seconds and sample rate
  void setTuningsContinuous(float Kp, float Ki_per_s, float Kd_seconds, float sampleRateHz);
  // compute: setpoint and measurement in same units (degrees).
  // returns control in angular velocity (deg/s) (float).
  float compute(float setpoint, float measurement, float dt_s);
  void reset();
  // expose current tunings
  void getTunings(float &out_kp, float &out_ki, float &out_kd);
private:
  float kp, ki, kd; // kp: proportional (unitless gain on degrees), ki: Ki_per_s, kd: Kd_seconds
  float integral;
  float prevError;
  float derivFiltered;
  float outMin, outMax; // outMin/outMax are in deg/s
  // derivative filter alpha (0..1)
  float d_alpha;
};


