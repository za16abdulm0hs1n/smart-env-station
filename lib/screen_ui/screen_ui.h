/**
 * @file screen_ui.h
 * @brief Minimal OLED UI for SSD1306 using U8x8 (text-only) mode.
 *
 * Design goals:
 *  - Very small RAM footprint (targets AVR/UNO and ESP32)
 *  - Clear, single-responsibility drawing functions (one page per method)
 *  - AVR-safe float formatting (avoid printf %f on AVR)
 *
 * Provided pages:
 *  - drawMenu(selected, names, count)
 *  - drawAll(...)                 — legacy composite page (interleaved value + enabled flag + footerSel)
 *  - drawAll(..., ledOn, gMag)    — extended composite page (LED + Accel, interleaved)
 *  - Scrollable per-sensor pages with On/Off/Back options for each sensor (interleaved value + enabled flag + footerSel)
 *
 * Usage:
 *  - Call begin(flip) once in setup()
 *  - Use drawMenu() in MENU state
 *  - Call one of the page methods in VIEW state
 */

#pragma once

#include <Arduino.h>
#include <U8x8lib.h>

/**
 * @class ScreenUi
 * @brief Minimal OLED user interface wrapper for SSD1306 using U8x8 text mode.
 *
 * U8x8 trades graphics for tiny RAM usage. Each draw* method fully redraws
 * its page; the UI logic (menu selection, timing) is handled outside.
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
   * @param flip If true, rotates display by 180° (useful if mounted upside-down).
   */
  void begin(bool flip = false);

  /**
   * @brief Draw a vertical menu of text items with a cursor.
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

  // -------- Composite pages --------

  /**
   * @brief Draw legacy composite page with multiple sensor readings and enabled flags.
   *
   * Shows temperature, humidity, pressure, light (raw ADC), and sound (RMS).
   * Each sensor value is interleaved with its enabled flag.
   * Options shown at bottom as On/Off/Back footer.
   *
   * @param tempC       Temperature in °C.
   * @param tempEnabled True if temperature sensor is enabled.
   * @param rhPct       Relative humidity (%).
   * @param rhEnabled   True if humidity sensor is enabled.
   * @param pressHpa    Pressure in hPa.
   * @param pressEnabled True if pressure sensor is enabled.
   * @param ldrRaw      Raw LDR ADC value.
   * @param ldrEnabled  True if light sensor is enabled.
   * @param rms         Sound RMS amplitude.
   * @param rmsEnabled  True if sound sensor is enabled.
   * @param footerSel   Selected footer option index (0=On,1=Off,2=Back).
   */
  void drawAll(float tempC, bool tempEnabled, float rhPct, bool rhEnabled,
               float pressHpa, bool pressEnabled, int ldrRaw, bool ldrEnabled,
               float rms, bool rmsEnabled,
               uint8_t footerSel);

  /**
   * @brief Draw extended composite page with LED state, accelerometer magnitude, and enabled flags.
   *
   * Extended variant that:
   *  - prints LDR as percentage (0..100)
   *  - shows LED state (ON/OFF)
   *  - includes accelerometer magnitude |g|
   *  - interleaves each sensor value with its enabled flag
   *
   * Options shown at bottom as On/Off/Back footer.
   *
   * @param tempC       Temperature in °C (NAN if unavailable).
   * @param tempEnabled True if temperature sensor is enabled.
   * @param rhPct       Relative humidity (%) (NAN if unavailable).
   * @param rhEnabled   True if humidity sensor is enabled.
   * @param pressHpa    Pressure in hPa (NAN if unavailable).
   * @param pressEnabled True if pressure sensor is enabled.
   * @param ldrPct      LDR light percentage [0..100].
   * @param ldrEnabled  True if light sensor is enabled.
   * @param rms         Sound RMS amplitude.
   * @param rmsEnabled  True if sound sensor is enabled.
   * @param ledOn       True if the local kit LED is ON.
   * @param ledEnabled  True if LED is enabled.
   * @param accelG      Acceleration magnitude in g (NAN if unavailable).
   * @param accelEnabled True if accelerometer is enabled.
   * @param footerSel   Selected footer option index (0=On,1=Off,2=Back).
   */
  void drawAll(float tempC, bool tempEnabled, float rhPct, bool rhEnabled,
               float pressHpa, bool pressEnabled, float ldrPct, bool ldrEnabled,
               float rms, bool rmsEnabled,
               bool ledOn, bool ledEnabled,
               float accelG, bool accelEnabled,
               uint8_t footerSel);

  // -------- Interactive sensor pages with interleaved value + enabled flag and On/Off/Back options --------

  /**
   * @brief Draw scrollable temperature/humidity page with On/Off/Back options.
   * Shows readings if enabled==true, else shows "OFF". Options shown at bottom.
   * @param tempC       Temperature in °C (NAN prints as "---").
   * @param tempEnabled True to show temperature value, false to show "OFF".
   * @param rhPct       Relative humidity %  (NAN prints as "---").
   * @param rhEnabled   True to show humidity value, false to show "OFF".
   * @param footerSel   Selected footer option index (0=On,1=Off,2=Back).
   */
  void drawTempHumPage(float tempC, bool tempEnabled, float rhPct, bool rhEnabled, uint8_t footerSel);

  /**
   * @brief Draw scrollable pressure page with On/Off/Back options.
   * Shows reading if enabled==true, else shows "OFF". Options shown at bottom.
   * @param pressHpa    Pressure in hPa (NAN prints as "---").
   * @param pressEnabled True to show pressure value, false to show "OFF".
   * @param footerSel   Selected footer option index (0=On,1=Off,2=Back).
   */
  void drawPressurePage(float pressHpa, bool pressEnabled, uint8_t footerSel);

  /**
   * @brief Draw scrollable light page with On/Off/Back options.
   * Shows reading if enabled==true, else shows "OFF". Options shown at bottom.
   * @param ldrValue    Light sensor value (float).
   * @param ldrEnabled  True to show light value, false to show "OFF".
   * @param footerSel   Selected footer option index (0=On,1=Off,2=Back).
   */
  void drawLightPage(float ldrValue, bool ldrEnabled, uint8_t footerSel);

  /**
   * @brief Draw scrollable sound page with On/Off/Back options.
   * Shows reading if enabled==true, else shows "OFF". Options shown at bottom.
   * @param rms         Sound RMS amplitude.
   * @param rmsEnabled  True to show sound value, false to show "OFF".
   * @param footerSel   Selected footer option index (0=On,1=Off,2=Back).
   */
  void drawSoundPage(float rms, bool rmsEnabled, uint8_t footerSel);

  /**
   * @brief Draw scrollable accelerometer page with On/Off/Back options.
   * Shows reading if enabled==true, else shows "OFF". Options shown at bottom.
   * @param accelG      Acceleration magnitude (g). Use NAN to indicate invalid.
   * @param accelEnabled True to show accelerometer value, false to show "OFF".
   * @param footerSel   Selected footer option index (0=On,1=Off,2=Back).
   */
  void drawAccelPage(float accelG, bool accelEnabled, uint8_t footerSel);

private:
  U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8_; ///< HW I2C SSD1306 driver (no reset pin)

  /**
   * @brief AVR-safe float-to-string helper (dtostrf-based).
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

  /**
   * @brief Draw footer with On/Off/Back options.
   * @param sel Selected footer option index (0=On,1=Off,2=Back).
   */
  void drawFooter(uint8_t sel);
};