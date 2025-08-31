#pragma once
#include <Arduino.h>

/**
 * @class Bmp280Sensor
 * @brief BMP280 wrapper for pressure (hPa) and temperature (°C).
 *
 * Provides a minimal interface for using the BMP280 sensor
 * through the Adafruit BMP280 library. Offers methods to
 * initialize the sensor and read pressure/temperature values.
 */
class Bmp280Sensor {
public:
  /**
   * @brief Default constructor.
   */
  Bmp280Sensor() {}

  /**
   * @brief Initialize I2C and the BMP280 sensor.
   *
   * Sets up communication with the BMP280 over I2C.
   *
   * @return true if initialization was successful, false otherwise.
   */
  bool begin();

  /**
   * @brief Read air pressure from BMP280.
   *
   * Retrieves the current air pressure in hectopascals (hPa).
   *
   * @return Pressure in hPa, or NAN if the reading fails.
   */
  float readPressureHpa();

  /**
   * @brief Read temperature from BMP280.
   *
   * Retrieves the current temperature in degrees Celsius.
   * Can be used for comparison with AHT20 sensor values.
   *
   * @return Temperature in °C.
   */
  float readTempC();

  /**
   * @brief Check if the sensor is operational.
   *
   * @return true if sensor initialized correctly, false otherwise.
   */
  bool isOk() const { return ok_; }

private:
  /// @brief Internal flag indicating whether the sensor is initialized.
  bool ok_ = false;
};