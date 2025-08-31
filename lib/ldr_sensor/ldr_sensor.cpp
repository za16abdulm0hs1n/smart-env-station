/**
 * @file ldr_sensor.cpp
 * @brief Implementation of the LdrSensor class.
 */

#include "ldr_sensor.h"
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
static constexpr int LDR_ADC_MAX = 4095; // 12-bit ESP32 ADC
#else
static constexpr int LDR_ADC_MAX = 1023; // 10-bit AVR ADC
#endif

/**
 * @brief Read raw ADC value from the LDR pin.
 *
 * Performs an @c analogRead() on the configured pin.
 *
 * @return Raw ADC value (0..1023 on 10-bit Arduino boards).
 */
int LdrSensor::readRaw() {
  return analogRead(_pin);
}

/**
 * @brief Read light level as percentage.
 *
 * Maps the raw ADC value (0..1023) into 0–100%.
 * - 0% = completely dark
 * - 100% = maximum brightness (saturation)
 *
 * Note: depending on circuit wiring, higher ADC values may correspond
 * to either more light or more darkness.
 *
 * @return Light intensity in percentage (0–100).
 */
float LdrSensor::readPercent() {
  int raw = analogRead(_pin);
  // integer rounding to nearest percent
  float pct = (100.0f * (float)raw) / (float)LDR_ADC_MAX;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

/**
 * @brief Check if environment is "dark" compared to a threshold.
 *
 * Reads the current raw ADC value and compares it against a threshold.
 *
 * @param threshold Raw ADC cutoff (default in header = 300).
 * @return true if raw < threshold (considered dark), false otherwise.
 */
bool LdrSensor::isDark(int threshold) {
  int raw = analogRead(_pin);
  return (raw < threshold);
}