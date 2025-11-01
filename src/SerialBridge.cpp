#include "SerialBridge.h"
#include "BLEHandler.h"
#include <Arduino.h>

SerialBridge::SerialBridge() {
  _buffer = "";
  _getPidRequested = false;
}

void SerialBridge::begin(unsigned long baud) {
  Serial.begin(baud);
}

void SerialBridge::printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  SET PID <kp> <ki> <kd>   -- set PID (floats)"));
  Serial.println(F("  GET PID                  -- show current PID"));
  Serial.println(F("  HELP"));
}

// simplistic parser: called in loop
bool SerialBridge::poll(PIDParams &paramsOut) {
  bool gotSet = false;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String line = _buffer;
      line.trim();
      _buffer = "";
      if (line.length() == 0) continue;
      // tokenize
      // transform to upper for command matching
      String up = line;
      up.toUpperCase();
      if (up.startsWith("SET PID")) {
        // parse three floats from original line (to preserve signs)
        float kp, ki, kd;
        int parsed = sscanf(line.c_str(), "SET PID %f %f %f", &kp, &ki, &kd);
        if (parsed == 3) {
          paramsOut.kp = kp;
          paramsOut.ki = ki;
          paramsOut.kd = kd;
          gotSet = true;
          Serial.printf("ACK SET PID %.6f %.6f %.6f\n", kp, ki, kd);
        } else {
          Serial.println("ERR SET PID requires three floats");
        }
      } else if (up == "GET PID" || up.startsWith("GET PID ")) {
        // request that main prints current PID (so access remains in controller)
        _getPidRequested = true;
      } else if (up == "HELP") {
        printHelp();
      } else {
        Serial.println("UNKNOWN CMD");
      }
    } else {
      _buffer += c;
      // guard buffer length
      if (_buffer.length() > 256) _buffer = _buffer.substring(_buffer.length()-256);
    }
  }
  return gotSet;
}

void SerialBridge::printCurrent(PIDController &pid) {
  float kp, ki, kd;
  pid.getTunings(kp, ki, kd);
  Serial.printf("KP: %.6f KI: %.6f KD: %.6f\n", kp, ki, kd);

}

bool SerialBridge::consumeGetPidRequest() {
  noInterrupts();
  bool v = _getPidRequested;
  _getPidRequested = false;
  interrupts();
  return v;
}
