/**
 * @file aht20_sensor.cpp
 * @brief Implementation of Aht20Sensor class (temperature & humidity via I²C).
 *
 * Follows datasheet commands for AHT20:
 *  - Init: check ACK
 *  - Trigger: send 0xAC 0x33 0x00
 *  - Read: 6 bytes → 20-bit humidity + 20-bit temperature
 */

#include "aht20_sensor.h"

#define AHT20_ADDR 0x38   ///< Default I²C address for AHT20

bool Aht20Sensor::begin() {
  Wire.begin();
  delay(40); // sensor startup time
  Wire.beginTransmission(AHT20_ADDR);
  if (Wire.endTransmission() != 0) {
    _ok = false;
    return false;
  }
  _ok = true;
  return true;
}

/**
 * @brief Send measurement trigger command to AHT20.
 * @return true if ACKed by device, false otherwise.
 */
bool Aht20Sensor::triggerMeasurement() {
  Wire.beginTransmission(AHT20_ADDR);
  Wire.write(0xAC); ///< Trigger measurement command (per datasheet)
  Wire.write(0x33); ///< Fixed second byte
  Wire.write(0x00); ///< Fixed third byte
  if (Wire.endTransmission() != 0) return false;
  delay(80); // typical measurement duration ~75 ms
  return true;
}

/**
 * @brief Read 6 raw data bytes and unpack 20-bit humidity + temperature.
 * @param[out] rawT 20-bit raw temperature data
 * @param[out] rawH 20-bit raw humidity data
 * @return true if read successful and 6 bytes received
 */
bool Aht20Sensor::readRaw(uint32_t &rawT, uint32_t &rawH) {
  Wire.requestFrom(AHT20_ADDR, 6u);
  if (Wire.available() < 6) return false;

  uint8_t data[6];
  for (uint8_t i = 0; i < 6; i++) data[i] = Wire.read();

  // Humidity: upper 20 bits across bytes 1–3
  uint32_t h = ((uint32_t)data[1] << 12) |
               ((uint32_t)data[2] << 4)  |
               (data[3] >> 4);

  // Temperature: lower 20 bits across bytes 3–5
  uint32_t t = (((uint32_t)data[3] & 0x0F) << 16) |
               ((uint32_t)data[4] << 8) |
               data[5];

  rawH = h;
  rawT = t;
  return true;
}

/**
 * @brief Read and convert temperature to °C.
 * @return Temperature in Celsius, or NAN if failed.
 */
float Aht20Sensor::readTempC() {
  if (!triggerMeasurement()) { _ok = false; return NAN; }
  uint32_t rawT, rawH;
  if (!readRaw(rawT, rawH)) { _ok = false; return NAN; }
  _ok = true;
  // Datasheet formula: T = (rawT / 2^20) * 200 - 50
  return ((float)rawT / 1048576.0f) * 200.0f - 50.0f;
}

/**
 * @brief Read and convert humidity to %RH.
 * @return Relative humidity [0..100], or NAN if failed.
 */
float Aht20Sensor::readHumidity() {
  if (!triggerMeasurement()) { _ok = false; return NAN; }
  uint32_t rawT, rawH;
  if (!readRaw(rawT, rawH)) { _ok = false; return NAN; }
  _ok = true;
  // Datasheet formula: RH = (rawH / 2^20) * 100
  return ((float)rawH / 1048576.0f) * 100.0f;
}