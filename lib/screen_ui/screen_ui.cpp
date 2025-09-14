/****
 * @file screen_ui.h
 * @brief Declaration of the ScreenUi class (SSD1306 OLED with U8x8).
 */



#include <Arduino.h>
#include <U8x8lib.h>
#include "screen_ui.h"


/**
 * @file screen_ui.cpp
 * @brief Implementation of the ScreenUi class (SSD1306 OLED with U8x8).
 */

#include <math.h>   // for isnan
#include <string.h> // for strncpy, strlcat

// ------------------ ctor ------------------

ScreenUi::ScreenUi()
: u8x8_(U8X8_PIN_NONE) {}

// ------------------ init ------------------

void ScreenUi::begin(bool flip) {
  u8x8_.begin();
  u8x8_.setPowerSave(0);
  u8x8_.setFlipMode(flip ? 1 : 0);
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
}

// ------------------ helpers ------------------

void ScreenUi::f2s(float v, uint8_t prec, char* out, size_t n) {
  dtostrf(v, 0, prec, out);
  out[n-1] = '\0';
}

void ScreenUi::line(uint8_t row, const char* text) {
  u8x8_.drawString(0, row, text);
}

void ScreenUi::kv(uint8_t row, const __FlashStringHelper* k, const char* v) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%s %s", (const char*)k, v);
  u8x8_.drawString(0, row, buf);
}

void ScreenUi::drawFooter(uint8_t sel) {
  // Footer options: On Off Back
  // Highlight option indicated by sel
  char foot[18];
  const char* options[3] = {"On", "Off", "Back"};
  char buf[18];
  size_t pos = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    if (i == sel) {
      int n = snprintf(buf, sizeof(buf), ">%s", options[i]);
      if (pos + n < sizeof(foot)) {
        memcpy(foot + pos, buf, n);
        pos += n;
      }
    } else {
      int n = snprintf(buf, sizeof(buf), " %s", options[i]);
      if (pos + n < sizeof(foot)) {
        memcpy(foot + pos, buf, n);
        pos += n;
      }
    }
    if (i < 2 && pos < sizeof(foot)) {
      foot[pos++] = ' ';
    }
  }
  if (pos >= sizeof(foot))
    pos = sizeof(foot) - 1;
  foot[pos] = '\0';

  u8x8_.setFont(u8x8_font_5x7_f);
  u8x8_.drawString(0, 7, foot);
}

// ------------------ menu ------------------

void ScreenUi::drawMenu(uint8_t selected,
                        const char* const* names,
                        uint8_t count) {
  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  line(0, "Select page:");

  const uint8_t ROWS = 5; // rows 2..6 are usable
  if (count == 0) {
    line(3, "(empty)");
    u8x8_.setFont(u8x8_font_5x7_f);
    line(7, "Knob=move  Btn=OK");
    return;
  }

  uint8_t start = 0;
  if (count > ROWS) {
    int s = (int)selected - (int)(ROWS / 2);
    if (s < 0) s = 0;
    int maxStart = (int)count - (int)ROWS;
    if (s > maxStart) s = maxStart;
    start = (uint8_t)s;
  }
  uint8_t end = start + ROWS;
  if (end > count) end = count;

  for (uint8_t idx = start, row = 0; idx < end; ++idx, ++row) {
    char ln[18];
    snprintf(ln, sizeof(ln), "%c %s", (idx == selected ? '>' : ' '), names[idx]);
    u8x8_.drawString(0, 2 + row, ln);
  }

  u8x8_.setFont(u8x8_font_5x7_f);
  char foot[18];
  snprintf(foot, sizeof(foot), "%u/%u  Knob=move", (unsigned)(selected + 1U), (unsigned)count);
  u8x8_.drawString(0, 7, foot);
}

// ------------------ pages ------------------

void ScreenUi::drawAll(float tempC, bool tempEnabled,
                       float rhPct, bool rhEnabled,
                       float pressHpa, bool pressEnabled,
                       float ldrPct, bool ldrEnabled,
                       float rms, bool rmsEnabled,
                       bool ledOn, bool ledEnabled,
                       float accelG, bool accelEnabled,
                       uint8_t footerSel) {
  char t[10], h[10], p[12], l[10], r[10], a[10];

  if (!tempEnabled || isnan(tempC)) { strncpy(t, "OFF", sizeof(t)); t[sizeof(t)-1]='\0'; }
  else                              { f2s(tempC, 1, t, sizeof(t)); }

  if (!rhEnabled || isnan(rhPct)) { strncpy(h, "OFF", sizeof(h)); h[sizeof(h)-1]='\0'; }
  else                            { f2s(rhPct, 0, h, sizeof(h)); }

  if (!pressEnabled || isnan(pressHpa)) { strncpy(p, "OFF", sizeof(p)); p[sizeof(p)-1]='\0'; }
  else                                  { f2s(pressHpa, 1, p, sizeof(p)); }

  if (!ldrEnabled || isnan(ldrPct)) { strncpy(l, "OFF", sizeof(l)); l[sizeof(l)-1]='\0'; }
  else                             { f2s(ldrPct, 0, l, sizeof(l)); }

  if (!rmsEnabled || isnan(rms)) { strncpy(r, "OFF", sizeof(r)); r[sizeof(r)-1]='\0'; }
  else                           { f2s(rms, 0, r, sizeof(r)); }

  if (!accelEnabled || isnan(accelG)) { strncpy(a, "OFF", sizeof(a)); a[sizeof(a)-1]='\0'; }
  else                                { f2s(accelG, 2, a, sizeof(a)); }

  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);

  line(0, "All Readings");

  char ln[18];
  // Temperature line
  if (strcmp(t, "OFF") == 0) {
    snprintf(ln, sizeof(ln), "T=OFF");
  } else {
    snprintf(ln, sizeof(ln), "T=%sC", t);
  }
  u8x8_.drawString(0, 2, ln);

  // Humidity line
  if (strcmp(h, "OFF") == 0) {
    snprintf(ln, sizeof(ln), "H=OFF");
  } else {
    snprintf(ln, sizeof(ln), "H=%s%%", h);
  }
  u8x8_.drawString(0, 3, ln);

  // Pressure line
  if (strcmp(p, "OFF") == 0) {
    snprintf(ln, sizeof(ln), "P=OFF");
  } else {
    snprintf(ln, sizeof(ln), "P=%shPa", p);
  }
  u8x8_.drawString(0, 4, ln);

  // Light line
  if (strcmp(l, "OFF") == 0) {
    snprintf(ln, sizeof(ln), "LDR=OFF");
  } else {
    snprintf(ln, sizeof(ln), "LDR=%s%%", l);
  }
  u8x8_.drawString(0, 5, ln);

  // Sound line
  if (strcmp(r, "OFF") == 0) {
    snprintf(ln, sizeof(ln), "Sound=OFF");
  } else {
    snprintf(ln, sizeof(ln), "Sound=%s%%", r);
  }
  u8x8_.drawString(0, 6, ln);

  // Accel line
  if (strcmp(a, "OFF") == 0) {
    snprintf(ln, sizeof(ln), "Accel=OFF");
  } else {
    snprintf(ln, sizeof(ln), "Accel=%s g", a);
  }
  u8x8_.drawString(0, 7, ln);
}

void ScreenUi::drawTempHumPage(float tempC, bool tempEnabled,
                               float rhPct, bool rhEnabled,
                               uint8_t footerSel) {
  char t[10], h[10];

  if (!tempEnabled || isnan(tempC)) { strncpy(t, "OFF", sizeof(t)); t[sizeof(t)-1]='\0'; }
  else                              { f2s(tempC, 1, t, sizeof(t)); }

  if (!rhEnabled || isnan(rhPct)) { strncpy(h, "OFF", sizeof(h)); h[sizeof(h)-1]='\0'; }
  else                            { f2s(rhPct, 0, h, sizeof(h)); }

  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  line(0, "Temp / Hum");

  char ln[18];
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  if (strcmp(t, "OFF") == 0) {
    snprintf(ln, sizeof(ln), "Temp: OFF");
  } else {
    snprintf(ln, sizeof(ln), "Temp: %sC", t);
  }
  u8x8_.drawString(0, 2, ln);

  if (strcmp(h, "OFF") == 0) {
    snprintf(ln, sizeof(ln), "Hum: OFF");
  } else {
    snprintf(ln, sizeof(ln), "Hum: %s%%", h);
  }
  u8x8_.drawString(0, 4, ln);
  u8x8_.setFont(u8x8_font_5x7_f);

  drawFooter(footerSel);
}

void ScreenUi::drawPressurePage(float pressHpa, bool pressEnabled,
                                uint8_t footerSel) {
  char p[12];
  if (!pressEnabled || isnan(pressHpa)) { strncpy(p, "OFF", sizeof(p)); p[sizeof(p)-1]='\0'; }
  else                                  { f2s(pressHpa, 1, p, sizeof(p)); }

  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  line(0, "Pressure");

  char ln[18];
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  if (strcmp(p, "OFF") == 0) {
    snprintf(ln, sizeof(ln), "OFF");
  } else {
    snprintf(ln, sizeof(ln), "%s hPa", p);
  }
  u8x8_.drawString(0, 3, ln);
  u8x8_.setFont(u8x8_font_5x7_f);

  drawFooter(footerSel);
}

void ScreenUi::drawLightPage(float ldrValue, bool ldrEnabled,
                             uint8_t footerSel) {
  char l[10];
  if (!ldrEnabled || isnan(ldrValue)) { strncpy(l, "OFF", sizeof(l)); l[sizeof(l)-1]='\0'; }
  else                                { f2s(ldrValue, 0, l, sizeof(l)); }

  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  line(0, "Light");

  char ln[18];
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  if (strcmp(l, "OFF") == 0) {
    snprintf(ln, sizeof(ln), "LDR: OFF");
  } else {
    snprintf(ln, sizeof(ln), "LDR: %s%%", l);
  }
  u8x8_.drawString(0, 3, ln);
  u8x8_.setFont(u8x8_font_5x7_f);

  drawFooter(footerSel);
}

void ScreenUi::drawSoundPage(float rms, bool rmsEnabled,
                             uint8_t footerSel) {
  char r[10];
  if (!rmsEnabled || isnan(rms)) { strncpy(r, "OFF", sizeof(r)); r[sizeof(r)-1]='\0'; }
  else                           { f2s(rms, 0, r, sizeof(r)); }

  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  line(0, "Sound");

  char ln[18];
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  if (strcmp(r, "OFF") == 0) {
    snprintf(ln, sizeof(ln), "Sound: OFF");
  } else {
    snprintf(ln, sizeof(ln), "Sound: %s%%", r);
  }
  u8x8_.drawString(0, 3, ln);
  u8x8_.setFont(u8x8_font_5x7_f);

  drawFooter(footerSel);
}

void ScreenUi::drawAccelPage(float accelG, bool accelEnabled,
                             uint8_t footerSel) {
  char a[10];
  if (!accelEnabled || isnan(accelG)) { strncpy(a, "OFF", sizeof(a)); a[sizeof(a)-1]='\0'; }
  else                                { f2s(accelG, 2, a, sizeof(a)); }

  u8x8_.clear();
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  line(0, "Accel");

  char ln[18];
  u8x8_.setFont(u8x8_font_chroma48medium8_r);
  if (strcmp(a, "OFF") == 0) {
    snprintf(ln, sizeof(ln), "Mag: OFF");
  } else {
    snprintf(ln, sizeof(ln), "Mag: %s g", a);
  }
  u8x8_.drawString(0, 3, ln);
  u8x8_.setFont(u8x8_font_5x7_f);

  drawFooter(footerSel);
}