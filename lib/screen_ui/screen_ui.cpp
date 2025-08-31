/**
 * @file screen_ui.cpp
 * @brief Implementation of the ScreenUi class (SSD1306 OLED with U8x8).
 */

#include "screen_ui.h"

// ------------------ ctor ------------------

/**
 * @brief Construct ScreenUi with default SSD1306 128x64 driver (I2C).
 *
 * Uses U8X8 hardware I2C; reset pin is unused.
 */
ScreenUi::ScreenUi()
: u8x8_(U8X8_PIN_NONE) {}

// ------------------ init ------------------

/**
 * @brief Initialize the OLED display.
 *
 * Sets up U8x8 driver, disables power save, applies rotation if requested,
 * and selects a default font.
 *
 * @param flip If true, rotates display by 180°.
 */
void ScreenUi::begin(bool flip) {
  u8x8_.begin();
  u8x8_.setPowerSave(0);
  u8x8_.setFlipMode(flip ? 1 : 0);
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
}

// ------------------ helpers ------------------

/**
 * @brief AVR-safe float-to-string conversion.
 *
 * Uses @c dtostrf internally to avoid lack of %f support in printf/snprintf on AVR.
 *
 * @param v    Float value to convert.
 * @param prec Number of decimal places.
 * @param out  Destination buffer.
 * @param n    Size of destination buffer.
 */
void ScreenUi::f2s(float v, uint8_t prec, char* out, size_t n) {
  dtostrf(v, 0, prec, out);
  out[n-1] = '\0';
}

/**
 * @brief Draw a string at the given row.
 * @param row 0-based row index (0–7).
 * @param text Null-terminated string to draw.
 */
void ScreenUi::line(uint8_t row, const char* text) {
  u8x8_.drawString(0, row, text);
}

/**
 * @brief Draw a "Key Value" formatted line.
 *
 * Uses snprintf to concatenate key and value with a space separator.
 *
 * @param row Row index (0–7).
 * @param k   Key string in PROGMEM (flash).
 * @param v   Value string in RAM.
 */
void ScreenUi::kv(uint8_t row, const __FlashStringHelper* k, const char* v) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%s %s", (const char*)k, v);
  u8x8_.drawString(0, row, buf);
}

// ------------------ menu ------------------

/**
 * @brief Draw a vertical menu with highlighting.
 *
 * Displays up to 5 items starting at row 2; highlights the selected index with '>'.
 *
 * @param selected Index of the highlighted entry.
 * @param names    Array of menu item strings.
 * @param count    Number of items in @p names.
 */
void ScreenUi::drawMenu(uint8_t selected,
                        const char* const* names,
                        uint8_t count) {
  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  line(0, "Select page:");
  for (uint8_t i = 0; i < count && i < 5; ++i) {
    char ln[18];
    snprintf(ln, sizeof(ln), "%c %s", (i == selected ? '>' : ' '), names[i]);
    u8x8_.drawString(0, 2 + i, ln);
  }
  u8x8_.setFont(u8x8_font_5x7_f);
  line(7, "Knob=move  Btn=OK");
}

// ------------------ pages ------------------

/**
 * @brief Draw composite page showing all sensor readings.
 *
 * @param tempC    Temperature in °C.
 * @param rhPct    Relative humidity (%).
 * @param pressHpa Pressure in hPa.
 * @param ldrRaw   Raw LDR ADC value.
 * @param rms      Sound RMS value.
 */
void ScreenUi::drawAll(float tempC, float rhPct, float pressHpa,
                       int ldrRaw, float rms) {
  char t[10], h[10], p[12], r[10], ln[18];
  f2s(tempC,   1, t, sizeof(t));
  f2s(rhPct,   0, h, sizeof(h));
  f2s(pressHpa,1, p, sizeof(p));
  f2s(rms,     0, r, sizeof(r));

  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  line(0, "All Readings");

  snprintf(ln, sizeof(ln), "T=%sC  H=%s%%", t, h); u8x8_.drawString(0, 2, ln);
  snprintf(ln, sizeof(ln), "P=%shPa", p);         u8x8_.drawString(0, 3, ln);
  snprintf(ln, sizeof(ln), "LDR=%4d", ldrRaw);    u8x8_.drawString(0, 4, ln);
  snprintf(ln, sizeof(ln), "RMS=%s", r);          u8x8_.drawString(0, 5, ln);

  u8x8_.setFont(u8x8_font_5x7_f);
  line(7, "Btn=Back");
}

/**
 * @brief Draw page with temperature and humidity.
 *
 * @param tempC Temperature in °C.
 * @param rhPct Relative humidity (%).
 */
void ScreenUi::drawTempHum(float tempC, float rhPct) {
  char t[10], h[10], ln[18];
  f2s(tempC, 1, t, sizeof(t));
  f2s(rhPct, 0, h, sizeof(h));

  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  line(0, "Temp / Hum");
  snprintf(ln, sizeof(ln), "Temp: %s C", t); u8x8_.drawString(0, 2, ln);
  snprintf(ln, sizeof(ln), "Hum:  %s %%", h); u8x8_.drawString(0, 3, ln);
  u8x8_.setFont(u8x8_font_5x7_f);
  line(7, "Btn=Back");
}

/**
 * @brief Draw page with pressure reading.
 * @param pressHpa Pressure in hPa.
 */
void ScreenUi::drawPressure(float pressHpa) {
  char p[12], ln[18];
  f2s(pressHpa, 1, p, sizeof(p));

  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  line(0, "Pressure");
  snprintf(ln, sizeof(ln), "%s hPa", p); u8x8_.drawString(0, 3, ln);
  u8x8_.setFont(u8x8_font_5x7_f);
  line(7, "Btn=Back");
}

/**
 * @brief Draw page with LDR light reading.
 * @param ldrRaw Raw ADC value from LDR.
 */
void ScreenUi::drawLight(int ldrRaw) {
  char ln[18];
  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  line(0, "Light");
  snprintf(ln, sizeof(ln), "LDR: %4d", ldrRaw); u8x8_.drawString(0, 3, ln);
  u8x8_.setFont(u8x8_font_5x7_f);
  line(7, "Btn=Back");
}

/**
 * @brief Draw page with sound RMS reading.
 * @param rms Sound RMS value.
 */
void ScreenUi::drawSound(float rms) {
  char r[10], ln[18];
  f2s(rms, 0, r, sizeof(r));
  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  line(0, "Sound");
  snprintf(ln, sizeof(ln), "RMS: %s", r); u8x8_.drawString(0, 3, ln);
  u8x8_.setFont(u8x8_font_5x7_f);
  line(7, "Btn=Back");
}