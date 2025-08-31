/**
 * @file io_devices.cpp
 * @brief Implementations for Button, Led, Potentiometer, and Buzzer helpers.
 */

#include "io_devices.h"

// ======================== Button ========================

/**
 * @brief Construct a Button object.
 * @param pin Digital input pin connected to the button.
 * @param activeHigh True if logic HIGH means "pressed", false if logic LOW means "pressed".
 * @param debounceMs Debounce interval in milliseconds.
 */
Button::Button(uint8_t pin, bool activeHigh, uint16_t debounceMs)
: _pin(pin),
  _activeHigh(activeHigh),
  _debounceMs(debounceMs),
  _lastRaw(false),
  _stable(false),
  _lastChange(0),
  _pressEdge(false),
  _releaseEdge(false) {}

/**
 * @brief Initialize the button pin.
 *
 * Configures pin as input. Assumes external pull-up/pull-down resistor
 * depending on wiring. Reads the initial raw state and sets stable state.
 */
void Button::begin() {
  pinMode(_pin, INPUT);
  bool raw = (digitalRead(_pin) == HIGH);
  if (!_activeHigh) raw = !raw;
  _lastRaw = raw;
  _stable  = raw;
  _lastChange = millis();
}

/**
 * @brief Update the button state with debounce.
 *
 * Call once per loop. Tracks raw changes, applies debounce timing,
 * and latches edge flags for pressed()/released().
 */
void Button::update() {
  bool raw = (digitalRead(_pin) == HIGH);
  if (!_activeHigh) raw = !raw;

  const unsigned long now = millis();

  // raw state changed -> restart debounce timer
  if (raw != _lastRaw) {
    _lastRaw    = raw;
    _lastChange = now;
  }

  // after debounce interval, accept new stable state and latch edge
  if ((now - _lastChange) >= _debounceMs) {
    if (raw != _stable) {
      if (raw) _pressEdge = true;
      else     _releaseEdge = true;
      _stable = raw;
    }
  }
}

/**
 * @brief Check if button was pressed (rising edge).
 * @return true once per press event.
 */
bool Button::pressed()  { bool e = _pressEdge;   _pressEdge   = false; return e; }

/**
 * @brief Check if button was released (falling edge).
 * @return true once per release event.
 */
bool Button::released() { bool e = _releaseEdge; _releaseEdge = false; return e; }


// ========================== Led =========================

/**
 * @brief Construct a Led object.
 * @param pin Digital output pin driving the LED.
 */
Led::Led(uint8_t pin)
: _pin(pin), _on(false) {}

/**
 * @brief Initialize LED pin as output and switch it off.
 */
void Led::begin() {
  pinMode(_pin, OUTPUT);
  off();
}

/**
 * @brief Switch LED on.
 */
void Led::on() {
  digitalWrite(_pin, HIGH);
  _on = true;
}

/**
 * @brief Switch LED off.
 */
void Led::off() {
  digitalWrite(_pin, LOW);
  _on = false;
}

/**
 * @brief Toggle LED state.
 */
void Led::toggle() {
  _on ? off() : on();
}


// ===================== Potentiometer ====================

/**
 * @brief Construct a Potentiometer object.
 * @param pin Analog input pin connected to the potentiometer.
 */
Potentiometer::Potentiometer(uint8_t pin)
: _pin(pin), _alpha(0.0f), _ema(0.0f) {}

/**
 * @brief Initialize potentiometer pin.
 *
 * Analog pins typically need no setup on Arduino.
 */
void Potentiometer::begin() {
  // No pinMode needed for analogRead() on most Arduino cores.
}

/**
 * @brief Read raw analog value.
 * @return ADC value (0..1023 on AVR, 0..4095 on ESP32).
 */
int Potentiometer::readRaw() {
  return analogRead(_pin);
}

/**
 * @brief Read analog value with exponential smoothing.
 *
 * Applies EMA with factor @c alpha. If alpha <= 0, returns raw value.
 *
 * @return Smoothed analog reading.
 */
float Potentiometer::readSmooth() {
  const int raw = analogRead(_pin);
  if (_alpha <= 0.0f) return static_cast<float>(raw);
  _ema = _alpha * static_cast<float>(raw) + (1.0f - _alpha) * _ema;
  return _ema;
}

/**
 * @brief Read analog value scaled to a custom range.
 * @param min Minimum of target range.
 * @param max Maximum of target range.
 * @return Scaled value in [min..max].
 */
float Potentiometer::readScaled(float min, float max) {
  const int   raw   = readRaw();
  const float ratio = static_cast<float>(raw) / 1023.0f;
  return min + (max - min) * ratio;
}


// ========================= Buzzer =======================

/**
 * @brief Construct a Buzzer object.
 * @param pin Digital output pin driving buzzer.
 * @param activeHigh True if HIGH = on, false if LOW = on.
 */
Buzzer::Buzzer(uint8_t pin, bool activeHigh)
: _pin(pin), _activeHigh(activeHigh), _on(false) {}

/**
 * @brief Initialize buzzer pin and set off.
 */
void Buzzer::begin() {
  pinMode(_pin, OUTPUT);
  off();
}

/**
 * @brief Turn buzzer on (active buzzers).
 */
void Buzzer::on() {
  digitalWrite(_pin, _activeHigh ? HIGH : LOW);
  _on = true;
}

/**
 * @brief Turn buzzer off.
 */
void Buzzer::off() {
  digitalWrite(_pin, _activeHigh ? LOW : HIGH);
  _on = false;
}

/**
 * @brief Toggle buzzer state.
 */
void Buzzer::toggle() {
  _on ? off() : on();
}

/**
 * @brief Generate a tone on a passive buzzer.
 * @param freq Frequency in Hz.
 * @param duration Duration in ms.
 */
void Buzzer::tone(uint16_t freq, unsigned long duration) {
  ::tone(_pin, freq, duration);
}

/**
 * @brief Stop tone generation on a passive buzzer.
 */
void Buzzer::noTone() {
  ::noTone(_pin);
}