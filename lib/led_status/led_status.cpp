/**
 * @file led_status.cpp
 * @brief Status LED control for Smart Environment Station (sensor board).
 *
 * Implements:
 *  - 5× sensor LEDs (red): Temp/Hum, Pressure, Light, Sound, Accel
 *  - TX activity LED (blue): short pulse when sending
 *  - ERROR LED (yellow): on when any error is flagged
 *  - POWER/heartbeat LED (green): slow blink to indicate the firmware is alive
 *
 * Notes:
 *  - Pin mappings come from @ref app_pins.h (PIN_LED_SENSOR1..5, PIN_LED_TX, PIN_LED_ERROR, PIN_LED_PWR).
 *  - All logic here is non-blocking; call ledsUpdate() periodically (e.g., every loop).
 */

#include <Arduino.h>
#include "app_pins.h"
#include "led_status.h"

// ----------------------------------------------------------------------------
// Internal state latches
// ----------------------------------------------------------------------------
static bool s_temp  = false;
static bool s_press = false;
static bool s_light = false;
static bool s_sound = false;
static bool s_accel = false;
static bool s_error = false;

// Heartbeat (green) state
static unsigned long hb_t0   = 0;
static bool          hb_state= false;

// TX pulse (blue) state
static unsigned long tx_until= 0;

/// @brief Drive a pin HIGH.
static inline void pinOn(uint8_t p)  { digitalWrite(p, HIGH); }
/// @brief Drive a pin LOW.
static inline void pinOff(uint8_t p) { digitalWrite(p, LOW); }

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

void ledsBegin() {
  // Configure all status LED pins as outputs and turn OFF
  const uint8_t pins[] = {
    PIN_LED_SENSOR1, PIN_LED_SENSOR2, PIN_LED_SENSOR3, PIN_LED_SENSOR4, PIN_LED_SENSOR5,
    PIN_LED_TX, PIN_LED_ERROR, PIN_LED_PWR
  };
  for (uint8_t pin : pins) { pinMode(pin, OUTPUT); pinOff(pin); }

  // Reset latches
  s_temp = s_press = s_light = s_sound = s_accel = false;
  s_error = false;

  // Reset timers
  hb_t0    = millis();
  hb_state = false;
  tx_until = 0;
}

void ledsSetSensorStatus(bool tempOk, bool pressOk, bool lightOk, bool soundOk, bool accelOk) {
  // Latch states
  s_temp  = tempOk;
  s_press = pressOk;
  s_light = lightOk;
  s_sound = soundOk;
  s_accel = accelOk;

  // Drive the 5 RED sensor LEDs according to the latched states
  digitalWrite(PIN_LED_SENSOR1, s_temp  ? HIGH : LOW);   // Temp/Hum
  digitalWrite(PIN_LED_SENSOR2, s_press ? HIGH : LOW);   // Pressure
  digitalWrite(PIN_LED_SENSOR3, s_light ? HIGH : LOW);   // Light
  digitalWrite(PIN_LED_SENSOR4, s_sound ? HIGH : LOW);   // Sound
  digitalWrite(PIN_LED_SENSOR5, s_accel ? HIGH : LOW);   // Accel
}

void ledsSetError(bool on) {
  s_error = on;
  digitalWrite(PIN_LED_ERROR, s_error ? HIGH : LOW);
}

void ledsPulseTx(uint16_t ms) {
  tx_until = millis() + ms;
  pinOn(PIN_LED_TX);
}

void ledsUpdate() {
  const unsigned long now = millis();

  // POWER/heartbeat (green): ~1 Hz blink
  if (now - hb_t0 >= 500) {
    hb_t0    = now;
    hb_state = !hb_state;
  }
  digitalWrite(PIN_LED_PWR, hb_state ? HIGH : LOW);

  // TX pulse (blue): auto-clear after the requested duration
  if (tx_until != 0 && now >= tx_until) {
    tx_until = 0;
    pinOff(PIN_LED_TX);
  }

  // ERROR and sensor LEDs are driven immediately by their setter functions.
}