/**
 * @file main.cpp
 * @brief Main application integrating I/O devices, sensors, OLED UI, status LEDs, and cloud.
 *
 * Features:
 *  - Potentiometer controls menu cursor; button selects/back (with LED+buzzer pulse).
 *  - LDR-based kit LED control (dark → ON, bright → OFF) with hysteresis.
 *  - Sound RMS computed continuously (no clap toggle).
 *  - OLED text UI with pages: All, Temp/Hum, Pressure, Light, Sound.
 *  - ESP32: ThingSpeak upload via HTTP or MQTT (guarded by USE_MQTT).
 *  - Status LEDs (sensor board):
 *      * 5× RED  : one per sensor (Temp/Hum, Pressure, Light, Sound, Accel)
 *      * 1× BLUE : TX activity (blink on successful send)
 *      * 1× YELLOW: Error indicator (ON if critical sensors fail)
 *      * 1× GREEN : Power/heartbeat (blinks ~1 Hz in ledsUpdate())
 *
 * Boards:
 *  - Arduino Uno (ATmega328P)
 *  - ESP32 NodeMCU-32S
 *
 * Pin mapping is centralized in @ref app_pins.h.
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>   ///< isnan/isfinite for sensor gating
#include "app_pins.h"          ///< Central pin mapping (UNO / ESP32)

#include <io_devices.h>        ///< Button, Led, Potentiometer, Buzzer
#include <ldr_sensor.h>        ///< LdrSensor (readRaw/readPercent)
#include <sound_sensor.h>      ///< SoundSensor (RMS producer)
#include <aht20_sensor.h>      ///< Aht20Sensor (Temp/Humidity)
#include <bmp280_sensor.h>     ///< Bmp280Sensor (Pressure)
#include <screen_ui.h>         ///< ScreenUi (U8x8 text-mode OLED)
#include <accel_sensor.h>     ///< AccelSensor (MPU-6050 |a| in g)
#include "led_status.h"        ///< Status LEDs (5× sensor, TX, ERR, PWR)

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  #include <WiFi.h>
  #include "cloud_config.h"           ///< USE_MQTT + TS_MIN_PERIOD_MS + credentials
  #include "cloud_thingspeak.h"       ///< HTTP sender: ts_post(...)
  #include "cloud_thingspeak_mqtt.h"  ///< MQTT sender: ts_mqtt_publish(...)
#endif

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#if USE_MQTT
/**
 * @brief Print the exact MQTT target details at boot for quick verification.
 *
 * Shows Channel ID, Write Key (masked on screen UI but full here for debug),
 * broker host:port, and the full publish topic so you can cross-check against
 * your ThingSpeak channel page.
 */
static void printMqttTarget()
{
  Serial.print(F("[TS] Channel: ")); Serial.println(TS_CHANNEL_ID);
  Serial.print(F("[TS] WriteKey: ")); Serial.println(TS_WRITE_KEY);
  Serial.print(F("[TS] Broker: "));  Serial.print(MQTT_HOST);
  Serial.print(F(":"));               Serial.println(MQTT_PORT);
  String topic = String("channels/") + TS_CHANNEL_ID + "/publish/" + TS_WRITE_KEY;
  Serial.print(F("[TS] Topic: "));   Serial.println(topic);
}
#endif
#endif


// ===================== Devices =====================

Led           led(PIN_LED);   ///< Kit LED (local ambient light rule)
Button        btn(PIN_BTN);   ///< User button (debounced)
Potentiometer pot(PIN_POT);   ///< Menu input
LdrSensor     ldr(PIN_LDR);   ///< Light sensor (LDR → %)
SoundSensor   snd(PIN_SND);   ///< Sound sensor (RMS)
Aht20Sensor   aht;            ///< AHT20 temperature/humidity
Bmp280Sensor  bmp;            ///< BMP280 pressure
Buzzer        buz(PIN_BUZ);   ///< Passive buzzer for press feedback
ScreenUi      ui;             ///< OLED UI
AccelSensor   accel;          ///< Accelerometer (magnitude in g)

// ===================== UI State =====================

/** @brief UI pages shown on OLED. */
enum Page : uint8_t {
  PAGE_ALL=0,     ///< Composite page
  PAGE_TEMP_HUM,  ///< Temperature & Humidity
  PAGE_PRESS,     ///< Pressure
  PAGE_LIGHT,     ///< Light (LDR %)
  PAGE_SOUND,     ///< Sound RMS
  PAGE_ACCEL,    ///< Acceleration magnitude
  PAGE_COUNT
};

static const char* PAGE_NAMES[PAGE_COUNT] = {
  "All","Temp/Hum","Pressure","Light","Sound","Accel"
};

/** @brief High-level UI states. */
enum UiState : uint8_t { ST_MENU, ST_VIEW };

static UiState uiState  = ST_MENU;
static Page    selected = PAGE_ALL;
static Page    viewing  = PAGE_ALL;
static uint8_t footerSel = 0; // Added static footer selection variable

// ===================== Button Feedback Pulse =====================

constexpr unsigned long PRESS_PULSE_MS = 600;  ///< LED/buzzer pulse duration
constexpr uint16_t      PRESS_TONE_HZ  = 2500; ///< Passive buzzer tone

static bool          pulseActive    = false;
static unsigned long pulseStartMs   = 0;

/**
 * @brief Start a short buzzer feedback pulse (non-blocking).
 */
static void startPressPulse() {
  buz.tone(PRESS_TONE_HZ, PRESS_PULSE_MS);
  pulseActive  = true;
  pulseStartMs = millis();
}

/**
 * @brief End the pulse when the timeout elapses; stop buzzer.
 */
static void updatePressPulse() {
  if (!pulseActive) return;
  if (millis() - pulseStartMs >= PRESS_PULSE_MS) {
    buz.noTone();
    pulseActive = false;
  }
}

// ===================== Helpers =====================

/**
 * @brief Map potentiometer raw ADC to menu index [0..count-1] with hysteresis.
 * @param raw   Raw ADC value.
 * @param count Number of menu items.
 */
static uint8_t pot_to_index_with_hysteresis(int raw, uint8_t count) {
  const int adcMax = 
  #if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    4096;  // ESP32 ADC1 is 0..4095
  #else
    1024;  // AVR 10-bit
  #endif
  const int bin  = max(1, adcMax / (int)count);
  const int dead = adcMax / 85; // ~1.2% deadband (ESP32≈48, UNO≈12)
  static int lastRaw = -32768;
  static uint8_t lastIdx = 0;

  if (abs(raw - lastRaw) < dead) return lastIdx;
  lastRaw = raw;

  uint8_t idx = (uint8_t)(raw / bin);
  if (idx >= count) idx = count - 1;
  lastIdx = idx;
  return idx;
}

/** @brief Check float validity (not NaN, finite). */
static inline bool validF(float v) { return !isnan(v) && isfinite(v); }

// ===================== Sensor Enable Flags =====================
// These let you disable sensors (LEDs will follow).
static bool enTempHum = true;
static bool enPress   = true;
static bool enLight   = true;
static bool enSound   = true;
static bool enAccel   = true;  // enable accel sensor

// ===================== Setup =====================


void setup() {


#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL); ///< Explicit I2C pins on ESP32
  Serial.println(USE_MQTT ? F("[MODE] MQTT") : F("[MODE] HTTP"));
  if (ts_connectWifi()) {
    Serial.print(F("WiFi IP: ")); Serial.println(WiFi.localIP());
#if USE_MQTT
    printMqttTarget();
#endif
  } else {
    Serial.println(F("WiFi not connected (will retry on post)."));
  }
#else
  Serial.begin(9600);
  Wire.begin();                 ///< Uno: SDA=A4, SCL=A5
#endif

  // Status LED panel (green heartbeat handled in ledsUpdate()).
  ledsBegin();

  // Local I/O
  led.begin();
  btn.begin();
  pot.begin();
  pot.setAlpha(0.2f);

  // Sensors
  ldr.begin();
  snd.begin();            // RMS calculation enabled; no clap actions
  (void)aht.begin();      // best-effort init (ok flag handled via reads)
  (void)bmp.begin();
  (void)accel.begin(0x19);  // best-effort init (MPU-6050 at 0x68; use 0x69 if AD0=HIGH)

  // OLED UI
  ui.begin(/*flip=*/true);
  ui.drawMenu(selected, PAGE_NAMES, PAGE_COUNT);

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (ts_connectWifi()) {
    Serial.print(F("WiFi IP: ")); Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("WiFi not connected (will retry on post)."));
  }
#endif
}

// ===================== Loop =====================

void loop() {
  // ---- Fast: button debounce, status LEDs heartbeat, sound RMS, LDR→kit LED ----
  btn.update();
  ledsUpdate();   // GREEN heartbeat + auto-clear BLUE TX pulse

  #if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  #if USE_MQTT
    ts_mqtt_loop();   // keep MQTT connection alive & receive publish_status
  #endif
  #endif

  static unsigned long tFast  = 0;
  static float         lastRms= 0;     // Used by UI/cloud
  static bool          ledDark= false;  // Hysteresis latch for kit LED

  if (millis() - tFast >= 5) {
    tFast = millis();

    // Keep sound RMS updated for UI/cloud (no LED action here).
    if (enSound) {
      snd.sample();
      if (snd.rmsReady()) lastRms = snd.lastRms();
    }

    // Kit LED rule: turn ON when dark enough, OFF when bright (hysteresis).
    const int DARK_ON_PCT  = 30;            // LED ON if ≤ 30%
    const int DARK_OFF_PCT = 40;            // LED OFF if ≥ 40%
    const int ldrPctNow    = ldr.readPercent();

    if (enLight) {
      if (!ledDark && ldrPctNow <= DARK_ON_PCT) { led.on();  ledDark = true;  }
      else if (ledDark && ldrPctNow >= DARK_OFF_PCT)         { led.off(); ledDark = false; }
    } else {
      led.off();
      ledDark = false;
    }
  }

  // ---- Potentiometer-driven menu selection ----
  if (uiState == ST_MENU) {
    static uint8_t lastSel = 255;
    const uint8_t potSel = pot_to_index_with_hysteresis(analogRead(PIN_POT), PAGE_COUNT);
    if (potSel != lastSel) {
      lastSel = potSel;
      selected = (Page)potSel;
      ui.drawMenu(selected, PAGE_NAMES, PAGE_COUNT);
    }
  }

  // ---- Periodic sensor read + UI refresh + LED status updates ----
  static unsigned long tUI = 0;
  static unsigned long okTempMs=0, okHumMs=0, okPressMs=0, okLightMs=0, okSoundMs=0, okAccelMs=0; // freshness timestamps
  const  unsigned long FRESH_MS = 3000;

  if (millis() - tUI >= 500) {
    tUI = millis();
    const unsigned long now = millis();

    // Read (skipping disabled sensors). Use NAN for disabled/not-read.
    float tC = NAN, rh = NAN, pHpa = NAN, aG = NAN;
    int   ldrPct = 0;

    if (enTempHum) {
      tC = aht.readTempC();
      rh = aht.readHumidity();
      if (validF(tC) && validF(rh)) { okTempMs = now; okHumMs = now; }
    }
    if (enPress) {
      pHpa = bmp.readPressureHpa();
      if (validF(pHpa)) okPressMs = now;
    }
    if (enLight) {
      ldrPct = ldr.readPercent();
      okLightMs = now;
    }
    if (enSound) {
      // RMS updated in fast path; just mark fresh usage here
      okSoundMs = now;
    }
    if (enAccel) {
      float gMag = accel.readG();       // returns NAN on failure
      if (!isnan(gMag)) { aG = gMag; okAccelMs = now; }
    }

    // Freshness → LED status (RED LEDs per-sensor).
    // Only light up LEDs for enabled AND fresh sensors, else OFF.
    const bool tempOk  = enTempHum && ((now - okTempMs ) <= FRESH_MS) && ((now - okHumMs) <= FRESH_MS);
    const bool pressOk = enPress   && ((now - okPressMs) <= FRESH_MS);
    const bool lightOk = enLight   && ((now - okLightMs) <= FRESH_MS);
    const bool soundOk = enSound   && ((now - okSoundMs) <= FRESH_MS);
    const bool accelOk = enAccel   && ((now - okAccelMs) <= FRESH_MS);

    ledsSetSensorStatus(tempOk, pressOk, lightOk, soundOk, accelOk);

    // Error LED rule (YELLOW): ON if a critical enabled sensor fails (Temp/Hum OR Pressure).
    const bool anyError = (enTempHum && !tempOk) || (enPress && !pressOk);
    ledsSetError(anyError);

    // UI page refresh
    if (uiState == ST_VIEW) {
      if (viewing == PAGE_ALL) {
        footerSel = 2;
        // No potentiometer scroll for PAGE_ALL: always set to 2 (Back)
      } else {
        // Potentiometer scroll for footerSel only if not PAGE_ALL
        footerSel = pot_to_index_with_hysteresis(analogRead(PIN_POT), 3);
      }
      switch (viewing) {
        case PAGE_ALL:
          ui.drawAll(
            tC, enTempHum,
            rh, enTempHum,
            pHpa, enPress,
            ldrPct, enLight,
            lastRms, enSound,
            led.state(), true,
            aG, enAccel,
            footerSel
          );
          break;
        case PAGE_TEMP_HUM: ui.drawTempHumPage(tC, enTempHum, rh, enTempHum, footerSel); break;
        case PAGE_PRESS:    ui.drawPressurePage(pHpa, enPress, footerSel); break;
        case PAGE_LIGHT:    ui.drawLightPage(ldrPct, enLight, footerSel); break;
        case PAGE_SOUND:    ui.drawSoundPage(snd.lastRmsPercent(), enSound, footerSel); break;
        case PAGE_ACCEL:    ui.drawAccelPage(aG, enAccel, footerSel); break;
        default: break;
      }
    }
  }

  // ---- Button press → feedback + state change ----
  if (btn.pressed()) {
    startPressPulse();
    if (uiState == ST_MENU) {
      uiState = ST_VIEW;
      viewing = selected;
      // Initialize footerSel appropriately on entry to ST_VIEW
      if (viewing == PAGE_ALL) {
        footerSel = 2;
      } else {
        footerSel = 0;
      }

      // Immediate draw on entry (fresh snapshots)
      const int   lNow = enLight   ? ldr.readPercent()     : 0;
      const float tNow = enTempHum ? aht.readTempC()       : NAN;
      const float hNow = enTempHum ? aht.readHumidity()    : NAN;
      const float pNow = enPress   ? bmp.readPressureHpa() : NAN;
      const float aNow = (enAccel ? accel.readG() : NAN);

      switch (viewing) {
        case PAGE_ALL:
          ui.drawAll(
            tNow, enTempHum,
            hNow, enTempHum,
            pNow, enPress,
            lNow, enLight,
            snd.lastRms(), enSound,
            led.state(), true,
            aNow, enAccel,
            footerSel
          );
          break;
        case PAGE_TEMP_HUM: ui.drawTempHumPage(tNow, enTempHum, hNow, enTempHum, footerSel); break;
        case PAGE_PRESS:    ui.drawPressurePage(pNow, enPress, footerSel); break;
        case PAGE_LIGHT:    ui.drawLightPage(lNow, enLight, footerSel); break;
        case PAGE_SOUND:    ui.drawSoundPage(snd.lastRmsPercent(), enSound, footerSel); break;
        case PAGE_ACCEL:    ui.drawAccelPage(aNow, enAccel, footerSel); break;
        default: break;
      }
    } else {
      // In ST_VIEW, check footerSel for action
      if (footerSel == 0) {
        // Enable current page sensor
        switch (viewing) {
          case PAGE_TEMP_HUM: enTempHum = true; break;
          case PAGE_PRESS:    enPress = true; break;
          case PAGE_LIGHT:    enLight = true; break;
          case PAGE_SOUND:    enSound = true; break;
          case PAGE_ACCEL:    enAccel = true; break;
          default: break;
        }
        // Redraw current page with updated enable flag
        const int   lNow = enLight   ? ldr.readPercent()     : 0;
        const float tNow = enTempHum ? aht.readTempC()       : NAN;
        const float hNow = enTempHum ? aht.readHumidity()    : NAN;
        const float pNow = enPress   ? bmp.readPressureHpa() : NAN;
        const float aNow = (enAccel ? accel.readG() : NAN);
        switch (viewing) {
          case PAGE_ALL:
            ui.drawAll(
              tNow, enTempHum,
              hNow, enTempHum,
              pNow, enPress,
              lNow, enLight,
              snd.lastRms(), enSound,
              led.state(), true,
              aNow, enAccel,
              footerSel
            );
            break;
          case PAGE_TEMP_HUM: ui.drawTempHumPage(tNow, enTempHum, hNow, enTempHum, footerSel); break;
          case PAGE_PRESS:    ui.drawPressurePage(pNow, enPress, footerSel); break;
          case PAGE_LIGHT:    ui.drawLightPage(lNow, enLight, footerSel); break;
          case PAGE_SOUND:    ui.drawSoundPage(snd.lastRmsPercent(), enSound, footerSel); break;
          case PAGE_ACCEL:    ui.drawAccelPage(aNow, enAccel, footerSel); break;
          default: break;
        }
      } else if (footerSel == 1) {
        // Disable current page sensor
        switch (viewing) {
          case PAGE_TEMP_HUM: enTempHum = false; break;
          case PAGE_PRESS:    enPress = false; break;
          case PAGE_LIGHT:    enLight = false; break;
          case PAGE_SOUND:    enSound = false; break;
          case PAGE_ACCEL:    enAccel = false; break;
          default: break;
        }
        // Redraw current page with updated enable flag
        const int   lNow = enLight   ? ldr.readPercent()     : 0;
        const float tNow = enTempHum ? aht.readTempC()       : NAN;
        const float hNow = enTempHum ? aht.readHumidity()    : NAN;
        const float pNow = enPress   ? bmp.readPressureHpa() : NAN;
        const float aNow = (enAccel ? accel.readG() : NAN);
        switch (viewing) {
          case PAGE_ALL:
            ui.drawAll(
              tNow, enTempHum,
              hNow, enTempHum,
              pNow, enPress,
              lNow, enLight,
              snd.lastRms(), enSound,
              led.state(), true,
              aNow, enAccel,
              footerSel
            );
            break;
          case PAGE_TEMP_HUM: ui.drawTempHumPage(tNow, enTempHum, hNow, enTempHum, footerSel); break;
          case PAGE_PRESS:    ui.drawPressurePage(pNow, enPress, footerSel); break;
          case PAGE_LIGHT:    ui.drawLightPage(lNow, enLight, footerSel); break;
          case PAGE_SOUND:    ui.drawSoundPage(snd.lastRmsPercent(), enSound, footerSel); break;
          case PAGE_ACCEL:    ui.drawAccelPage(aNow, enAccel, footerSel); break;
          default: break;
        }
      } else if (footerSel == 2) {
        // Return to menu
        uiState = ST_MENU;
        ui.drawMenu(selected, PAGE_NAMES, PAGE_COUNT);
      }
    }
  }

  updatePressPulse(); // end buzzer/LED pulse when timed out

// ---- Cloud upload (ESP32 only) ----
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  /**
   * ThingSpeak fields:
   *  f1 = Temp (°C), f2 = Hum (%), f3 = Pressure (hPa), f4 = Light (%),
   *  f5 = Sound RMS, f6 = Kit LED (0/1).
   * Blue TX LED is pulsed on success; Yellow error if send fails.
   */
  static unsigned long tsLast = 0;
  if (millis() - tsLast >= TS_MIN_PERIOD_MS) {
    tsLast = millis();

    const int   ldrPctUp = enLight   ? ldr.readPercent()     : -999;
    const float tCup     = enTempHum ? aht.readTempC()       : -999;
    const float rhUp     = enTempHum ? aht.readHumidity()    : -999;
    const float pUp      = enPress   ? bmp.readPressureHpa() : -999;
    const bool  ledOn    = led.state();
    const int   rmsUp    = enSound   ? snd.lastRmsPercent()  : -999;

  #if USE_MQTT
    // Publish a ThingSpeak update via MQTT.
    // NOTE: ts_mqtt_loop() is already called every iteration earlier in loop().
    const bool ok = ts_mqtt_publish(tCup, rhUp, pUp, ldrPctUp, rmsUp, ledOn);
    if (ok) {
      ledsPulseTx();
      Serial.println(F("MQTT publish sent"));
    } else {
      Serial.println(F("MQTT publish FAILED"));
      ledsSetError(true);
    }
  #else
    int httpCode = -1;
    const bool ok = ts_post(tCup, rhUp, pUp, ldrPctUp, rmsUp, ledOn, httpCode);
    if (ok) { ledsPulseTx(); Serial.println(F("ThingSpeak post: OK")); }
    else    { Serial.print(F("ThingSpeak post: FAIL (HTTP ")); Serial.print(httpCode); Serial.println(F(")")); ledsSetError(true); }
  #endif
  }
#endif

  delay(1); // cooperative pacing
}