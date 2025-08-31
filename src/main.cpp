/**
 * @file main.cpp
 * @brief Main application integrating I/O devices, sensors, and OLED UI.
 *
 * Features:
 *  - Potentiometer controls menu cursor
 *  - Button (debounced) selects/back with LED + passive-buzzer pulse
 *  - Clap detection via SoundSensor toggles LED
 *  - Pages: All, Temp+Hum, Pressure, Light, Sound (RMS)
 *
 * Boards:
 *  - Arduino Uno (ATmega328P)
 *  - ESP32 NodeMCU-32S
 * Pins are defined in @ref app_pins.h.
 */

#include <Arduino.h>
#include <Wire.h>
#include "app_pins.h"          ///< Central pin mapping (UNO / ESP32)

#include <io_devices.h>        ///< Button, Led, Potentiometer, Buzzer
#include <ldr_sensor.h>        ///< LdrSensor
#include <sound_sensor.h>      ///< SoundSensor (RMS + clap)
#include <aht20_sensor.h>      ///< Aht20Sensor
#include <bmp280_sensor.h>     ///< Bmp280Sensor
#include <screen_ui.h>         ///< ScreenUi (U8x8 text-mode OLED)

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  #include <WiFi.h>
  #include <cloud_config.h>           ///< USE_MQTT + TS_MIN_PERIOD_MS + creds
  #include "cloud_thingspeak.h"       ///< HTTP sender (ts_post)
  #include "cloud_thingspeak_mqtt.h"  ///< MQTT sender (ts_mqtt_publish)
#endif

// ----------------------- Device Objects ------------------------

Led           led(PIN_LED);   ///< LED digital output
Button        btn(PIN_BTN);   ///< User button (debounced)
Potentiometer pot(PIN_POT);   ///< Potentiometer input
LdrSensor     ldr(PIN_LDR);   ///< Light sensor
SoundSensor   snd(PIN_SND);   ///< Sound sensor (RMS + clap)
Aht20Sensor   aht;            ///< AHT20 temperature/humidity
Bmp280Sensor  bmp;            ///< BMP280 pressure
Buzzer        buz(PIN_BUZ);   ///< Passive buzzer
ScreenUi      ui;             ///< OLED user interface

// ----------------------- UI State Machine ----------------------

/**
 * @enum Page
 * @brief Menu pages shown on the OLED.
 */
enum Page : uint8_t {
  PAGE_ALL=0,     ///< Composite page with all readings
  PAGE_TEMP_HUM,  ///< Temperature and humidity
  PAGE_PRESS,     ///< Atmospheric pressure
  PAGE_LIGHT,     ///< Light intensity
  PAGE_SOUND,     ///< Sound RMS level
  PAGE_COUNT      ///< Number of pages
};

/// Human-readable page names for @ref Page
static const char* PAGE_NAMES[PAGE_COUNT] = { "All", "Temp/Hum", "Pressure", "Light", "Sound" };

/**
 * @enum UiState
 * @brief High-level UI states.
 */
enum UiState : uint8_t { ST_MENU, ST_VIEW };

static UiState uiState = ST_MENU;   ///< Current UI state
static Page    selected = PAGE_ALL; ///< Currently highlighted menu item
static Page    viewing  = PAGE_ALL; ///< Page being displayed

// ----------------------- Press Feedback ------------------------

constexpr unsigned long PRESS_PULSE_MS = 300;  ///< Feedback pulse duration [ms]
constexpr uint16_t      PRESS_TONE_HZ  = 1000; ///< Passive buzzer tone [Hz]

static bool          pulseActive    = false;  ///< Whether feedback pulse is active
static unsigned long pulseStartMs   = 0;      ///< Start time of current pulse
static bool          ledStateBefore = false;  ///< LED state before pulse

/**
 * @brief Start LED + buzzer feedback pulse (non-blocking).
 */
static void startPressPulse() {
  ledStateBefore = led.state();
  led.on();
  buz.tone(PRESS_TONE_HZ, PRESS_PULSE_MS);
  pulseActive  = true;
  pulseStartMs = millis();
}

/**
 * @brief Update and end pulse when elapsed.
 */
static void updatePressPulse() {
  if (!pulseActive) return;
  if (millis() - pulseStartMs >= PRESS_PULSE_MS) {
    if (ledStateBefore) led.on(); else led.off();
    buz.noTone();
    pulseActive = false;
  }
}

// ----------------------- Helpers -------------------------------

/**
 * @brief Map potentiometer ADC to menu index [0..count-1] with hysteresis.
 *
 * @param raw   Raw ADC reading
 * @param count Number of menu items
 * @return Menu index [0..count-1]
 */
static uint8_t pot_to_index_with_hysteresis(int raw, uint8_t count) {
  const int bin  = max(1, 1024 / (int)count);
  const int dead = 12; // deadband
  static int lastRaw = -32768;
  static uint8_t lastIdx = 0;
  if (abs(raw - lastRaw) < dead) return lastIdx;
  lastRaw = raw;
  uint8_t idx = (uint8_t)(raw / bin);
  if (idx >= count) idx = count - 1;
  lastIdx = idx;
  return idx;
}

// ----------------------- Setup --------------------------------

/**
 * @brief Initialize serial, I2C, devices, sensors, UI, and (ESP32) Wi-Fi try.
 */
void setup() {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL); ///< ESP32: explicit I2C pin selection
  Serial.println(USE_MQTT ? F("[MODE] MQTT") : F("[MODE] HTTP"));
#else
  Serial.begin(9600);
  Wire.begin();                 ///< Uno: SDA=A4, SCL=A5
#endif

  led.begin();
  btn.begin();
  pot.begin(); pot.setAlpha(0.2f);
  ldr.begin();
  snd.begin(); snd.setClapThreshold(200.0f);
  buz.begin();

  aht.begin();
  bmp.begin();

  ui.begin(/*flip=*/true);
  ui.drawMenu(selected, PAGE_NAMES, PAGE_COUNT);

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  // Optional: connect now; cloud helpers will retry later if it fails.
  if (ts_connectWifi()) {
    Serial.print(F("WiFi IP: ")); Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("WiFi not connected (will retry on post)."));
  }
#endif
}

// ----------------------- Loop ---------------------------------

/**
 * @brief Main loop: handles input, sensors, UI, feedback, and (ESP32) cloud.
 */
void loop() {
  // --- Fast tasks: button debounce, sound sampling ---
  btn.update();

  static unsigned long tSamp = 0;
  static float lastRms = 0;  // used by UI and cloud
  if (millis() - tSamp >= 5) {
    tSamp = millis();
    bool clap = snd.sample();
    if (snd.rmsReady()) lastRms = snd.lastRms();
    if (clap) led.toggle();  ///< LED toggle on clap
  }

  // --- Potentiometer: menu navigation ---
  uint8_t potSel = pot_to_index_with_hysteresis(analogRead(PIN_POT), PAGE_COUNT);
  if (uiState == ST_MENU) {
    static uint8_t lastSel = 255;
    if (potSel != lastSel) {
      lastSel = potSel;
      selected = (Page)potSel;
      ui.drawMenu(selected, PAGE_NAMES, PAGE_COUNT);
    }
  }

  // --- Periodic sensor read + UI refresh ---
  static unsigned long tUI = 0;
  if (millis() - tUI >= 500) {
    tUI = millis();
    int   ldrPct = ldr.readPercent();   // 0..100 %
    float tC     = aht.readTempC();
    float rh     = aht.readHumidity();
    float pHpa   = bmp.readPressureHpa();

    if (uiState == ST_VIEW) {
      switch (viewing) {
        case PAGE_ALL:      ui.drawAll(tC, rh, pHpa, ldrPct, lastRms); break;
        case PAGE_TEMP_HUM: ui.drawTempHum(tC, rh); break;
        case PAGE_PRESS:    ui.drawPressure(pHpa); break;
        case PAGE_LIGHT:    ui.drawLight(ldrPct); break;
        case PAGE_SOUND:    ui.drawSound(lastRms); break;
        default: break;
      }
    }
  }

  // --- Button press: feedback + navigation ---
  if (btn.pressed()) {
    startPressPulse();

    if (uiState == ST_MENU) {
      uiState = ST_VIEW;
      viewing = selected;

      // Immediate draw when entering page
      int   lNow = ldr.readPercent();
      float tNow = aht.readTempC();
      float hNow = aht.readHumidity();
      float pNow = bmp.readPressureHpa();
      switch (viewing) {
        case PAGE_ALL:      ui.drawAll(tNow, hNow, pNow, lNow, lastRms); break;
        case PAGE_TEMP_HUM: ui.drawTempHum(tNow, hNow); break;
        case PAGE_PRESS:    ui.drawPressure(pNow); break;
        case PAGE_LIGHT:    ui.drawLight(lNow); break;
        case PAGE_SOUND:    ui.drawSound(lastRms); break;
        default: break;
      }
    } else {
      uiState = ST_MENU;
      ui.drawMenu(selected, PAGE_NAMES, PAGE_COUNT);
    }
  }

  // --- End pulse if elapsed ---
  updatePressPulse();

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  /**
   * @brief Periodic ThingSpeak upload (respects free-tier rate).
   *
   * Fields:
   *  f1=Temp (°C), f2=Hum (%), f3=Press (hPa), f4=Light (%), f5=Sound RMS, f6=LED (0/1)
   */
  static unsigned long tsLast = 0;
  if (millis() - tsLast >= TS_MIN_PERIOD_MS) {
    tsLast = millis();

    const int   ldrPct = ldr.readPercent();
    const float tC     = aht.readTempC();
    const float rh     = aht.readHumidity();
    const float pHpa   = bmp.readPressureHpa();
    const bool  ledOn  = led.state();

  #if USE_MQTT
    const bool ok = ts_mqtt_publish(tC, rh, pHpa, ldrPct, lastRms, ledOn);
    Serial.println(ok ? F("MQTT publish sent") : F("MQTT publish FAILED"));
  #else
    int httpCode = -1;
    const bool ok = ts_post(tC, rh, pHpa, ldrPct, lastRms, ledOn, httpCode);
    Serial.print(F("ThingSpeak post: "));
    Serial.print(ok ? F("OK") : F("FAIL"));
    Serial.print(F(" (HTTP "));
    Serial.print(httpCode);
    Serial.println(F(")"));
  #endif
  }
#endif

  delay(1); // cooperative pacing
}