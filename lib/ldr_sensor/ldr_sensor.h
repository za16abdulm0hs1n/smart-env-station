/**
 * @file ldr_sensor.h
 * @brief Light-dependent resistor (LDR) sensor wrapper for Arduino.
 *
 * Encapsulates an analog photoresistor input. Provides:
 * - Raw ADC value reading (0–1023 on 10-bit Arduino boards).
 * - Percentage mapping for light level (0% = dark, 100% = bright).
 * - Simple darkness check against a configurable threshold.
 *
 * Typical usage:
 * @code
 *   LdrSensor ldr(A3);
 *   ldr.begin();
 *   int raw = ldr.readRaw();
 *   float percent = ldr.readPercent();
 *   if (ldr.isDark(400)) {
 *     // take action when dark
 *   }
 * @endcode
 */

#pragma once
#include <Arduino.h>

/**
 * @class LdrSensor
 * @brief Helper class for reading a Light-dependent resistor (LDR).
 *
 * Provides raw ADC values, percentage scaling, and threshold-based
 * darkness detection. Designed for simple light-sensing tasks.
 */
class LdrSensor {
public:
  /**
   * @brief Construct an LDR sensor instance.
   * @param pin Analog input pin connected to the LDR voltage divider (e.g., A3).
   */
  explicit LdrSensor(uint8_t pin) : _pin(pin) {}

  /**
   * @brief Initialize the sensor (symmetry placeholder).
   *
   * Included for API consistency with other device classes.
   * Currently does nothing; analog pins need no explicit setup.
   */
  void begin() {}

  /**
   * @brief Read raw ADC value from the LDR.
   * @return Raw ADC reading (0 = 0 V, 1023 = VCC on 10-bit Arduino).
   */
  int readRaw();

  /**
   * @brief Read light level as percentage.
   *
   * Maps the raw ADC value (0–1023) into a normalized range of 0–100%.
   * Interpretation depends on module wiring:
   * - In some circuits higher raw values = more light.
   * - In others, higher raw values = more darkness.
   *
   * @return Light intensity percentage (0% = dark, 100% = bright).
   */
  float readPercent();

  /**
   * @brief Check if environment is "dark".
   *
   * Compares the raw ADC value against a threshold. Defaults to 300,
   * but can be adjusted based on lighting conditions and wiring.
   *
   * @param threshold Raw ADC cutoff (0–1023). Default = 300.
   * @return true if raw < threshold (considered dark), false otherwise.
   */
  bool isDark(int threshold = 300);

private:
  uint8_t _pin; ///< Analog pin assigned to the LDR sensor
};