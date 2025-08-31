#pragma once
#include <Arduino.h>

/**
 * @file io_devices.h
 * @brief Simple I/O helper classes for Arduino projects:
 *        Button (debounced), Led, Potentiometer (EMA smoothing), and Buzzer.
 *
 * Each class wraps a small, focused behavior with an easy API and is documented
 * for automatic Doxygen generation.
 */

/**
 * @class Button
 * @brief Debounced digital input with edge detection.
 *
 * Provides stable button state (pressed/released) using time-based debouncing,
 * plus one-shot edge events via pressed() and released(). Supports active-high
 * or active-low wiring.
 */
class Button {
public:
  /**
   * @brief Construct a debounced button.
   * @param pin         Digital pin the button is connected to.
   * @param activeHigh  true if logic HIGH means pressed; false for active-low.
   * @param debounceMs  Debounce time window in milliseconds.
   */
  Button(uint8_t pin, bool activeHigh = true, uint16_t debounceMs = 30);

  /**
   * @brief Configure pin mode and initialize internal state.
   *
   * Sets the pin to INPUT (assumes external pull-up/down as wired).
   * Call this once in setup().
   */
  void begin();

  /**
   * @brief Update debouncer and edge detection.
   *
   * Call this frequently (e.g., each loop iteration). Reads the raw pin,
   * applies debouncing, and updates edge flags for pressed() / released().
   */
  void update();

  /**
   * @brief Current stable logical state (pressed).
   * @return true if the debounced state is "down/pressed".
   */
  bool isDown() const { return _stable; }

  /**
   * @brief Current stable logical state (released).
   * @return true if the debounced state is "up/released".
   */
  bool isUp()   const { return !_stable; }

  /**
   * @brief Rising edge detector (fires once when button becomes pressed).
   * @return true exactly once per press transition; false otherwise.
   */
  bool pressed();

  /**
   * @brief Falling edge detector (fires once when button is released).
   * @return true exactly once per release transition; false otherwise.
   */
  bool released();

private:
  uint8_t  _pin;              ///< GPIO pin number
  bool     _activeHigh;       ///< Button logic: true=HIGH means pressed
  uint16_t _debounceMs;       ///< Debounce interval in milliseconds
  bool     _lastRaw;          ///< Last raw (un-debounced) read
  bool     _stable;           ///< Current debounced stable state
  unsigned long _lastChange;  ///< Timestamp of last raw change (millis)
  bool     _pressEdge;        ///< Latched flag for press edge
  bool     _releaseEdge;      ///< Latched flag for release edge
};


/**
 * @class Led
 * @brief Simple digital output helper for an LED.
 *
 * Wraps pin mode, on/off/toggle, and cached state.
 */
class Led {
public:
  /**
   * @brief Construct an LED on a given digital pin.
   * @param pin Digital pin connected to the LED.
   */
  explicit Led(uint8_t pin);

  /**
   * @brief Configure pin as OUTPUT and default to OFF.
   */
  void begin();

  /**
   * @brief Set LED ON (writes HIGH).
   */
  void on();

  /**
   * @brief Set LED OFF (writes LOW).
   */
  void off();

  /**
   * @brief Toggle LED state.
   */
  void toggle();

  /**
   * @brief Read cached LED state.
   * @return true if LED is ON, false if OFF.
   */
  bool state() const { return _on; }

private:
  uint8_t _pin;   ///< GPIO pin number
  bool    _on;    ///< Cached on/off state
};


/**
 * @class Potentiometer
 * @brief Analog input with optional EMA smoothing and value scaling.
 *
 * Provides raw ADC reads, an exponential moving average (EMA) smoother,
 * and a helper to scale readings into an arbitrary [min,max] range.
 */
class Potentiometer {
public:
  /**
   * @brief Construct a potentiometer on the given analog-capable pin.
   * @param pin Analog input pin (e.g., A0).
   */
  explicit Potentiometer(uint8_t pin);

  /**
   * @brief Initialize if needed (placeholder for symmetry).
   */
  void  begin();

  /**
   * @brief Read raw ADC value.
   * @return Raw ADC reading (e.g., 0..1023 on 10-bit AVR).
   */
  int   readRaw();

  /**
   * @brief Read smoothed value using EMA filter.
   *
   * Formula: @f$ \text{EMA} \leftarrow \alpha \cdot x + (1-\alpha)\cdot \text{EMA} @f$
   * - @c alpha = 0.0 → no smoothing (equivalent to raw)
   * - Typical @c alpha : 0.1..0.3 for knobs/UI inputs
   *
   * @return Smoothed reading in raw ADC units.
   */
  float readSmooth();

  /**
   * @brief Read and scale to an application range.
   *
   * Maps the (optionally smoothed) ADC value into [min,max].
   *
   * @param min Lower bound of output range.
   * @param max Upper bound of output range.
   * @return Scaled value in [min,max].
   */
  float readScaled(float min, float max);

  /**
   * @brief Set EMA smoothing factor.
   * @param a Smoothing alpha in [0.0, 1.0]. Higher = smoother, slower response.
   */
  void  setAlpha(float a) { _alpha = a; }

private:
  uint8_t _pin;      ///< Analog input pin
  float   _alpha;    ///< EMA smoothing factor
  float   _ema;      ///< EMA accumulator
};


/**
 * @class Buzzer
 * @brief Simple digital or tone-based buzzer output.
 *
 * Supports active buzzers (on/off) or passive buzzers (tone generation).
 * - Active buzzer: use on()/off()/toggle() to control sound.
 * - Passive buzzer: use tone(freq, duration) and noTone() to play/stop tones.
 */
class Buzzer {
public:
  /**
   * @brief Construct a buzzer on a given pin.
   * @param pin Digital pin connected to the buzzer.
   * @param activeHigh true if HIGH drives buzzer on (active buzzer polarity).
   */
  explicit Buzzer(uint8_t pin, bool activeHigh = true);

  /**
   * @brief Initialize buzzer pin as OUTPUT (defaults to OFF).
   */
  void begin();

  /**
   * @brief Turn buzzer ON (for active buzzer modules).
   */
  void on();

  /**
   * @brief Turn buzzer OFF (for active buzzer modules).
   */
  void off();

  /**
   * @brief Toggle buzzer state (active buzzer only).
   */
  void toggle();

  /**
   * @brief Play tone on passive buzzer.
   * @param freq Frequency in Hz (e.g., 1000 = 1 kHz).
   * @param duration Duration in ms (0 = continuous until noTone()).
   */
  void tone(uint16_t freq, unsigned long duration = 0);

  /**
   * @brief Stop tone playback (passive buzzer only).
   */
  void noTone();

  /**
   * @brief Query ON/OFF state (for active buzzer).
   * @return true if currently ON.
   */
  bool state() const { return _on; }

private:
  uint8_t _pin;       ///< GPIO pin for buzzer
  bool    _activeHigh;///< Polarity for active buzzer
  bool    _on;        ///< Cached state for active buzzer
};