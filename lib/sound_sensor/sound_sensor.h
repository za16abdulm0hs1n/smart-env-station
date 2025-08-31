/**
 * @file sound_sensor.h
 * @brief Analog sound sensor wrapper (raw signal → RMS, simple clap detection).
 *
 * Usage:
 *  - Call sample() every ~5–10 ms (e.g., from loop).
 *  - Every ~100 ms, rmsReady() becomes true and lastRms() provides the new RMS value.
 *  - Tune clap detection with setClapThreshold() depending on environment.
 *
 * Notes:
 *  - RMS is calculated over a sliding window (default ~100 ms).
 *  - DC offset is removed with an exponential moving average (EMA).
 *  - Threshold-based clap detection is very simple: a single RMS peak above threshold.
 */

#pragma once
#include <Arduino.h>

/**
 * @class SoundSensor
 * @brief Analog microphone/sound sensor wrapper.
 *
 * Provides:
 *  - Raw analog reads
 *  - DC removal via EMA
 *  - RMS calculation over a configurable window
 *  - Simple clap detection via RMS threshold
 */
class SoundSensor {
public:
  /**
   * @brief Construct sound sensor on a given analog pin.
   * @param pin Analog input pin (e.g., A2).
   */
  explicit SoundSensor(uint8_t pin) : _pin(pin) {}

  /**
   * @brief Initialize sensor (placeholder, analog pins need no setup).
   */
  void begin() { /* analog pin needs no special init */ }

  /**
   * @brief Take one sound sample.
   *
   * Should be called periodically (every 5–10 ms). Updates DC-removed signal,
   * accumulates RMS window, and triggers clap detection.
   *
   * @return true if a clap was detected on this sample.
   */
  bool sample();

  /**
   * @brief Check if a new RMS value is ready.
   * @return true once per window (e.g., ~100 ms) when RMS is updated.
   */
  bool rmsReady() const { return _rmsReady; }

  /**
   * @brief Get last computed RMS value.
   *
   * Only valid when rmsReady() returned true in the previous loop.
   *
   * @return RMS amplitude in ADC counts.
   */
  float lastRms() const { return _lastRms; }

  /**
   * @brief Set alpha for DC removal EMA filter.
   *
   * Range: 0..1, close to 1 = slower drift removal (default = 0.995).
   *
   * @param a New alpha value.
   */
  void setDcAlpha(float a) { _dcAlpha = a; }

  /**
   * @brief Set RMS window size in samples.
   *
   * Default = 20 samples (~100 ms if sample() is called every 5 ms).
   *
   * @param n Number of samples per window (min = 1).
   */
  void setWindow(uint8_t n) { _winN= (n == 0 ? 1 : n); }

  /**
   * @brief Set clap detection threshold.
   *
   * Threshold is compared against RMS value. Typical range: 150–250.
   *
   * @param th RMS threshold (ADC counts).
   */
  void setClapThreshold(float th) { _clapTh = th; }

private:
  uint8_t _pin;              ///< Analog input pin

  // DC removal + RMS accumulation
  float _dc = 0.0f;          ///< EMA of raw (DC component)
  float _dcAlpha = 0.995f;   ///< EMA factor for DC removal
  float _accSq = 0.0f;       ///< Sum of squares for RMS calculation
  uint8_t _k = 0;            ///< Samples accumulated in current window
  uint8_t _winN = 20;           ///< Window size (samples per RMS update)

  // Outputs
  bool  _rmsReady = false;   ///< Flag: RMS ready once per window
  float _lastRms  = 0.0f;    ///< Last RMS value (ADC counts)

  // Clap detection
  float _clapTh = 200.0f;    ///< Threshold for clap detection
};