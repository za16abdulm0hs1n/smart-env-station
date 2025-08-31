#pragma once
#include <Arduino.h>

/**
 * @file cloud_thingspeak.h
 * @brief ThingSpeak client helper for ESP32.
 *
 * Provides two functions:
 *  - @ref ts_connectWifi : ensures Wi-Fi is connected (joins if needed).
 *  - @ref ts_post        : posts sensor data to ThingSpeak via HTTP POST.
 *
 * @note Only compiled for ESP32. On other platforms, this header will fail
 *       with an error directive to prevent accidental use.
 */

#if defined(Esp32) || defined(ARDUINO_ARCH_ESP32)

/**
 * @brief Ensure ESP32 is connected to Wi-Fi.
 *
 * - If already connected, returns immediately.
 * - If not connected, attempts to connect using credentials
 *   defined in @ref cloud_config.h.
 *
 * @return true if Wi-Fi is connected, false otherwise.
 */
bool ts_connectWifi();

/**
 * @brief Post one update to ThingSpeak.
 *
 * Sends sensor readings using HTTP POST to the `/update` endpoint of
 * ThingSpeak. The function blocks briefly during the HTTP transaction.
 *
 * @param tC       Temperature in Celsius.
 * @param rh       Relative humidity (%).
 * @param pHpa     Atmospheric pressure (hPa).
 * @param ldr      Light sensor raw ADC value.
 * @param rms      Sound RMS value.
 * @param httpCode [out] Parsed HTTP status code (200 = OK).
 *
 * @return true if HTTP transaction succeeded and ThingSpeak
 *         returned code 200 OK, false otherwise.
 */
// cloud_thingspeak.h
bool ts_post(float tC, float rh, float pHpa, int ldr, float rms, bool ledState, int &httpCode);

#else
  #error "ThingSpeak client is only available for ESP32 builds."
#endif