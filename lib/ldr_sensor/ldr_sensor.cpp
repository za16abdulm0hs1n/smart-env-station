/****
 * @file ldr_sensor.cpp
 * @brief Implementation of the LdrSensor class.
 */

#include "ldr_sensor.h"
#if defined(ESP32)
static constexpr int LDR_ADC_MAX = 4095; // 12-bit ESP32 ADC
#else
static constexpr int LDR_ADC_MAX = 1023; // 10-bit AVR ADC
#endif

static constexpr int RAW_MIN = 0;
static constexpr int RAW_MAX = 10;

/**
 * @brief Read raw ADC value from the LDR pin.
 *
 * Performs an @c analogRead() on the configured pin.
 *
 * @return Raw ADC value (0..1023 on 10-bit AVR boards, 0..4095 on ESP32).
 */
int LdrSensor::readRaw() {
  return analogRead(_pin);
}

/**
 * @brief Read light level as percentage.
 *
 * Maps the raw ADC value into 0–100% using calibrated RAW_MIN and RAW_MAX values.
 * Calibration uses RAW_MIN/RAW_MAX which should be tuned based on dark/bright readings.
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
  int pct = map(raw, RAW_MIN, RAW_MAX, 0, 100);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

/**
 * @brief Check if environment is "dark" compared to a threshold.
 *
 * Reads the current light level as a percentage and compares it against a threshold.
 *
 * @param threshold Percentage cutoff (0–100). If the measured light percent is less than this value, it is considered dark.
 * @return true if measured percent < threshold (considered dark), false otherwise.
 */
bool LdrSensor::isDark(int threshold) {
  float pct = readPercent();
  return (pct < threshold);
}