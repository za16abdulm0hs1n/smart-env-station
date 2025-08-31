#include "bmp280_sensor.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>

/// @brief Global BMP280 instance (I2C connection).
static Adafruit_BMP280 bmp; 

/**
 * @brief Initialize I2C and configure the BMP280 sensor.
 *
 * Tries both common I2C addresses (0x76, 0x77).
 * If successful, applies normal mode with oversampling
 * and filtering settings for stable pressure/temperature readings.
 *
 * @return true if sensor initialized correctly, false otherwise.
 */
bool Bmp280Sensor::begin() {
  Wire.begin();
  ok_ = bmp.begin(0x76) || bmp.begin(0x77); // common BMP280 addresses
  if (ok_) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,   ///< temperature oversampling
                    Adafruit_BMP280::SAMPLING_X16,  ///< pressure oversampling
                    Adafruit_BMP280::FILTER_X16,    ///< IIR filter
                    Adafruit_BMP280::STANDBY_MS_125 ///< standby time
    );
  }
  return ok_;
}

/**
 * @brief Read pressure from BMP280.
 *
 * Returns current air pressure converted from Pascals (as provided
 * by the Adafruit library) to hectopascals (hPa).
 *
 * @return Pressure in hPa, or NAN if sensor not ready or read fails.
 */
float Bmp280Sensor::readPressureHpa() {
  if (!ok_) return NAN;
  float p = bmp.readPressure(); // in Pascals
  if (isnan(p)) return NAN;
  return p / 100.0f;            // convert Pa → hPa
}

/**
 * @brief Read temperature from BMP280.
 *
 * Reads the temperature in Celsius directly from the BMP280.
 *
 * @return Temperature in °C, or NAN if sensor not ready.
 */
float Bmp280Sensor::readTempC() {
  if (!ok_) return NAN;
  return bmp.readTemperature(); // °C
}