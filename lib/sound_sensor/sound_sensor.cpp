#include "sound_sensor.h"

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  static constexpr int SND_ADC_MAX = 4095; // ESP32 12-bit ADC
#else
  static constexpr int SND_ADC_MAX = 1023; // AVR/UNO 10-bit ADC
#endif

/**
 * @brief Take one sample from the sound sensor.
 *
 * Workflow:
*  1. Reads raw ADC value (0–SND_ADC_MAX) and normalizes to 10‑bit (0–1023) for portability.
 *  2. Applies DC removal using an exponential moving average (EMA).
 *  3. Squares the AC component and accumulates it for RMS calculation.
 *  4. Once enough samples (_N) are collected:
 *     - Computes RMS value.
 *     - Resets accumulation.
 *     - Sets _rmsReady flag true.
 *     - Checks against clap threshold.
 *
 * @return true if a clap was detected in this RMS window,
 *         false otherwise (or if still accumulating).
 */
bool SoundSensor::sample() {
  _rmsReady = false;

  // --- Step 1: Read analog (0..SND_ADC_MAX), normalize to 10‑bit scale (0..1023) ---
  int raw = analogRead(_pin);
  float x = static_cast<float>(raw);
  // Normalize so thresholds/RMS match AVR behavior even on ESP32 12‑bit ADC
  const float kNorm = 1023.0f / static_cast<float>(SND_ADC_MAX);
  x *= kNorm;

  // --- Step 2: Remove DC component via EMA ---
  _dc = _dcAlpha * _dc + (1.0f - _dcAlpha) * x;
  float ac = x - _dc; ///< AC signal (centered around 0)

  // --- Step 3: Accumulate squared AC for RMS ---
  _accSq += ac * ac;
  _k++;

  // --- Step 4: Check if RMS window complete ---
  if (_k >= _winN) {
    _lastRms = sqrtf(_accSq / _winN); ///< RMS over window
    _accSq   = 0.0f;
    _k       = 0;
    _rmsReady = true;

    // Clap event if RMS exceeds threshold
    return (_lastRms >= _clapTh);
  }

  return false; // Still accumulating, no clap event
}