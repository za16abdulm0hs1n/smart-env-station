/**
 * @file accel_sensor.h
 * @brief Minimal I2C accelerometer wrapper (e.g., MPU6050) for ESP32/AVR.
 *
 * Interface:
 *  - begin() : init I2C device at 0x68; returns true on success
 *  - readG() : read acceleration magnitude in g (sqrt(ax^2+ay^2+az^2)), NAN on failure
 *  - isOk()  : true if last read succeeded
 *
 * Notes:
 *  - Uses Wire. SDA/SCL pins come from app_pins.h (Wire.begin(PIN_SDA, PIN_SCL)).
 *  - No heavy external library → keeps footprint small.
 */
#pragma once
#include <Arduino.h>
#include <Wire.h>

class AccelSensor {
public:
  AccelSensor() {}

  /// Initialize the device (wake up from sleep).
  bool begin(uint8_t addr = 0x68);

  /// Read magnitude in g; returns NAN on failure.
  float readG();

  /// Last read status
  bool isOk() const { return _ok; }

private:
  uint8_t _addr = 0x68;
  bool    _ok   = false;

  bool readRaw(int16_t &ax, int16_t &ay, int16_t &az);
  bool write8(uint8_t reg, uint8_t val);
};