// Modified main.cpp — accumulator-based fixed-timestep catch-up loop
#include "BotController.h"
#include "SerialBridge.h"
#include "Config.h"

BotController controller;
SerialBridge serialBridge;
unsigned long lastMicros = 0;
unsigned long accumMicros = 0;
const unsigned long tickMicros = (1000000UL / CONTROL_LOOP_HZ);
const int MAX_CATCHUP_TICKS = 5;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(50);
  Serial.println("SBR minimal starting...");
  controller.begin();
  serialBridge.begin(SERIAL_BAUD);
  lastMicros = micros();
}

void loop() {
  unsigned long now = micros();
  unsigned long elapsed = now - lastMicros;
  lastMicros = now;
  accumMicros += elapsed;

  int iterations = 0;
  // catch-up loop: run one or more fixed-size control ticks until we are up-to-date
  while (accumMicros >= tickMicros && iterations < MAX_CATCHUP_TICKS) {
    float dt = (tickMicros) / 1000000.0f; // seconds per fixed tick
    controller.update(dt);
    accumMicros -= tickMicros;
    iterations++;
  }

  // poll serial commands without blocking
  PIDParams p;
  if (serialBridge.poll(p)) {
    controller.requestPidParams((PIDParams&)p);
    Serial.printf("Requested PID apply from serial: kp=%.4f ki=%.6f kd=%.6f\n", p.kp, p.ki, p.kd);
  }

  // respond to GET PID requests
  if (serialBridge.consumeGetPidRequest()) {
    controller.printCurrentPid();
  }

  // yield to background tasks
  delay(0);
}


