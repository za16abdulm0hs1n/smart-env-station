/**
 * @file aht20_sensor.h
 * @brief AHT20 temperature & humidity sensor wrapper.
 *
 * Provides high-level API for reading temperature (°C) and humidity (%RH)
 * from the AHT20 sensor via I²C.
 */

#pragma once
#include <Arduino.h>
#include <Wire.h>

/**
 * @class Aht20Sensor
 * @brief Simple wrapper class for the AHT20 sensor.
 *
 * Responsibilities:
 *  - Initialize communication (I²C).
 *  - Trigger and read measurements.
 *  - Provide last-read status via isOk().
 */
class Aht20Sensor {
public:
  /// Default constructor.
  Aht20Sensor() {}

  /**
   * @brief Initialize I²C and check for sensor presence.
   * @return true if initialization successful, false otherwise.
   */
  bool begin();

  /**
   * @brief Read temperature from sensor.
   * @return Temperature in Celsius, or NAN if read failed.
   */
  float readTempC();

  /**
   * @brief Read relative humidity from sensor.
   * @return Humidity in %RH, or NAN if read failed.
   */
  float readHumidity();

  /**
   * @brief Indicates if the last read was successful.
   * @return true if last operation succeeded, false if failed.
   */
  bool isOk() const { return _ok; }

private:
  bool _ok = false; ///< Status flag indicating if the last read was successful.

  /**
   * @brief Send command to trigger a measurement on the AHT20.
   * @return true if command sent successfully, false otherwise.
   */
  bool triggerMeasurement();

  /**
   * @brief Read raw temperature and humidity data from the sensor.
   * @param[out] rawT Raw temperature value (20-bit).
   * @param[out] rawH Raw humidity value (20-bit).
   * @return true if data successfully read, false otherwise.
   */
  bool readRaw(uint32_t &rawT, uint32_t &rawH);
};