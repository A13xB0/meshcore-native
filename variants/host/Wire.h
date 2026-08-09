// No I2C bus on a host.
//
// MeshCore's RTC and sensor helpers include this unconditionally and then find
// nothing on the bus, which is the same thing a board with no RTC fitted
// reports. Answering "no device" is a real answer; refusing to compile is not.
#pragma once

#include <stdint.h>

class TwoWire {
 public:
  void begin() {}
  void begin(int, int) {}
  void beginTransmission(uint8_t) {}
  uint8_t endTransmission() { return 2; }  // 2: address NACK — nothing there
  uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
  int available() { return 0; }
  int read() { return -1; }
  size_t write(uint8_t) { return 1; }
  void setClock(uint32_t) {}
};

extern TwoWire Wire;
