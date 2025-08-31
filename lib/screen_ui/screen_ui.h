/**
 * @file screen_ui.h
 * @brief Minimal OLED UI for SSD1306 using U8x8 (text-only) mode.
 *
 * Designed for RAM-constrained boards (e.g. Arduino Uno).
 * Provides a simple menu system and multiple pages for sensor data.
 *
 * Features:
 *  - begin(flip) to initialize and rotate screen 0°/180°
 *  - drawMenu(selectedIdx, names[], count) to render a text menu
 *  - drawAll() composite page with multiple readings
 *  - drawTempHum(), drawPressure(), drawLight(), drawSound() for individual pages
 *
 * Notes:
 *  - Uses U8x8 (not U8g2) to minimize SRAM usage.
 *  - Avoids %f in printf/snprintf on AVR by using dtostrf helper.
 */

#pragma once

#include <Arduino.h>
#include <U8x8lib.h>

/**
 * @class ScreenUi
 * @brief Minimal OLED user interface wrapper for SSD1306 using U8x8 text mode.
 *
 * Provides a lightweight menu and page-based UI suitable for low-memory microcontrollers.
 */
class ScreenUi {
public:
  /**
   * @brief Construct ScreenUi instance.
   *
   * Uses hardware I2C interface with default SSD1306 128x64 driver.
   */
  ScreenUi();

  /**
   * @brief Initialize OLED and power it on.
   * @param flip If true, rotates display by 180° (useful if screen is mounted upside-down).
   */
  void begin(bool flip = false);

  /**
   * @brief Draw a vertical menu of text items.
   *
   * Displays each entry in @p names; places '>' cursor at @p selected index.
   *
   * @param selected Index of the highlighted menu entry.
   * @param names    Array of menu item strings.
   * @param count    Number of menu items.
   */
  void drawMenu(uint8_t selected,
                const char* const* names,
                uint8_t count);

  /**
   * @brief Draw composite page with multiple sensor readings.
   *
   * Shows temperature, humidity, pressure, light (LDR raw), and sound (RMS).
   *
   * @param tempC    Temperature in °C.
   * @param rhPct    Relative humidity (%).
   * @param pressHpa Pressure in hPa.
   * @param ldrRaw   Raw LDR ADC value.
   * @param rms      Sound RMS amplitude.
   */
  void drawAll(float tempC, float rhPct, float pressHpa,
               int ldrRaw, float rms);

  /**
   * @brief Draw page showing temperature and humidity.
   * @param tempC Temperature in °C.
   * @param rhPct Relative humidity (%).
   */
  void drawTempHum(float tempC, float rhPct);

  /**
   * @brief Draw page showing atmospheric pressure.
   * @param pressHpa Pressure in hPa.
   */
  void drawPressure(float pressHpa);

  /**
   * @brief Draw page showing light level.
   * @param ldrRaw Raw LDR ADC value.
   */
  void drawLight(int ldrRaw);

  /**
   * @brief Draw page showing sound level.
   * @param rms RMS amplitude.
   */
  void drawSound(float rms);

private:
  U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8_; ///< Hardware I2C SSD1306 driver (no reset pin)

  /**
   * @brief AVR-safe float-to-string helper.
   *
   * Uses dtostrf to avoid %f support in printf on AVR.
   *
   * @param v    Float value to convert.
   * @param prec Number of decimal places.
   * @param out  Destination char buffer.
   * @param n    Buffer size in bytes.
   */
  static void f2s(float v, uint8_t prec, char* out, size_t n);

  /**
   * @brief Draw a single line of text.
   * @param row Row index (0-7).
   * @param text Null-terminated string to display.
   */
  void line(uint8_t row, const char* text);

  /**
   * @brief Draw "Key: Value" formatted line.
   * @param row Row index (0-7).
   * @param k   Key stored in flash (PROGMEM).
   * @param v   Value string (RAM).
   */
  void kv(uint8_t row, const __FlashStringHelper* k, const char* v);
};