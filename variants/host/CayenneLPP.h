// Cayenne LPP, enough of it for the repeater's telemetry advert.
//
// MeshCore's own test mock is smaller than the repeater needs. This encodes the
// same wire format — channel, type, then a big-endian value at the type's own
// scale — so a telemetry packet built here is one a real receiver would decode.
#pragma once

// The LPP data types MeshCore's telemetry uses. Values are Cayenne's own, from
// the IPSO object registry — they go on the wire, so they are not ours to pick.
#ifndef LPP_TEMPERATURE
#define LPP_DIGITAL_INPUT 0
#define LPP_DIGITAL_OUTPUT 1
#define LPP_ANALOG_INPUT 2
#define LPP_ANALOG_OUTPUT 3
#define LPP_GENERIC_SENSOR 100
#define LPP_LUMINOSITY 101
#define LPP_PRESENCE 102
#define LPP_TEMPERATURE 103
#define LPP_RELATIVE_HUMIDITY 104
#define LPP_ACCELEROMETER 113
#define LPP_BAROMETRIC_PRESSURE 115
#define LPP_VOLTAGE 116
#define LPP_CURRENT 117
#define LPP_FREQUENCY 118
#define LPP_PERCENTAGE 120
#define LPP_ALTITUDE 121
#define LPP_CONCENTRATION 125
#define LPP_POWER 128
#define LPP_DISTANCE 130
#define LPP_ENERGY 131
#define LPP_DIRECTION 132
#define LPP_UNIXTIME 133
#define LPP_GYROMETER 134
#define LPP_COLOUR 135
#define LPP_GPS 136
#define LPP_SWITCH 142
#define LPP_POLYLINE 240

#define LPP_DIGITAL_INPUT_SIZE 1
#define LPP_DIGITAL_OUTPUT_SIZE 1
#define LPP_ANALOG_INPUT_SIZE 2
#define LPP_ANALOG_OUTPUT_SIZE 2
#define LPP_GENERIC_SENSOR_SIZE 4
#define LPP_LUMINOSITY_SIZE 2
#define LPP_PRESENCE_SIZE 1
#define LPP_TEMPERATURE_SIZE 2
#define LPP_RELATIVE_HUMIDITY_SIZE 1
#define LPP_ACCELEROMETER_SIZE 6
#define LPP_BAROMETRIC_PRESSURE_SIZE 2
#define LPP_VOLTAGE_SIZE 2
#define LPP_CURRENT_SIZE 2
#define LPP_FREQUENCY_SIZE 4
#define LPP_PERCENTAGE_SIZE 1
#define LPP_ALTITUDE_SIZE 2
#define LPP_CONCENTRATION_SIZE 2
#define LPP_POWER_SIZE 2
#define LPP_DISTANCE_SIZE 4
#define LPP_ENERGY_SIZE 4
#define LPP_DIRECTION_SIZE 2
#define LPP_UNIXTIME_SIZE 4
#define LPP_GYROMETER_SIZE 6
#define LPP_COLOUR_SIZE 3
#define LPP_GPS_SIZE 9
#define LPP_SWITCH_SIZE 1
#define LPP_MIN_POLYLINE_SIZE 8

#define LPP_DIGITAL_INPUT_MULT 1
#define LPP_DIGITAL_OUTPUT_MULT 1
#define LPP_ANALOG_INPUT_MULT 100
#define LPP_ANALOG_OUTPUT_MULT 100
#define LPP_GENERIC_SENSOR_MULT 1
#define LPP_LUMINOSITY_MULT 1
#define LPP_PRESENCE_MULT 1
#define LPP_TEMPERATURE_MULT 10
#define LPP_RELATIVE_HUMIDITY_MULT 2
#define LPP_ACCELEROMETER_MULT 1000
#define LPP_BAROMETRIC_PRESSURE_MULT 10
#define LPP_VOLTAGE_MULT 100
#define LPP_CURRENT_MULT 1000
#define LPP_FREQUENCY_MULT 1
#define LPP_PERCENTAGE_MULT 1
#define LPP_ALTITUDE_MULT 1
#define LPP_CONCENTRATION_MULT 1
#define LPP_POWER_MULT 1
#define LPP_DISTANCE_MULT 1000
#define LPP_ENERGY_MULT 1000
#define LPP_DIRECTION_MULT 1
#define LPP_UNIXTIME_MULT 1
#define LPP_GYROMETER_MULT 100
#define LPP_COLOUR_MULT 1
#define LPP_GPS_LAT_LON_MULT 10000
#define LPP_GPS_ALT_MULT 100
#define LPP_SWITCH_MULT 1

#define LPP_ERROR_OK 0
#define LPP_ERROR_OVERFLOW 1
#define LPP_ERROR_UNKOWN_TYPE 2
#endif

#include <stdint.h>
#include <string.h>

#define LPP_TEMPERATURE 103
#define LPP_VOLTAGE 116
#define LPP_RELATIVE_HUMIDITY 104
#define LPP_BAROMETRIC_PRESSURE 115

class CayenneLPP {
 public:
  explicit CayenneLPP(uint8_t size) : cap_(size) {
    buf_ = new uint8_t[size];
    len_ = 0;
  }
  ~CayenneLPP() { delete[] buf_; }

  void reset() { len_ = 0; }
  uint8_t getSize() const { return len_; }
  uint8_t* getBuffer() { return buf_; }

  // 0.01 V per count, big endian, as the specification defines it.
  uint8_t addVoltage(uint8_t channel, float v) {
    return add2(channel, LPP_VOLTAGE, (int16_t)(v * 100));
  }
  // 0.1 degrees per count.
  uint8_t addTemperature(uint8_t channel, float c) {
    return add2(channel, LPP_TEMPERATURE, (int16_t)(c * 10));
  }
  uint8_t addRelativeHumidity(uint8_t channel, float pct) {
    if (len_ + 3 > cap_) return 0;
    buf_[len_++] = channel;
    buf_[len_++] = LPP_RELATIVE_HUMIDITY;
    buf_[len_++] = (uint8_t)(pct * 2);
    return len_;
  }
  uint8_t addBarometricPressure(uint8_t channel, float hpa) {
    return add2(channel, LPP_BAROMETRIC_PRESSURE, (int16_t)(hpa * 10));
  }

 private:
  uint8_t add2(uint8_t channel, uint8_t type, int16_t value) {
    if (len_ + 4 > cap_) return 0;
    buf_[len_++] = channel;
    buf_[len_++] = type;
    buf_[len_++] = (uint8_t)(value >> 8);
    buf_[len_++] = (uint8_t)(value & 0xFF);
    return len_;
  }
  uint8_t* buf_ = nullptr;
  uint8_t cap_ = 0;
  uint8_t len_ = 0;
};
