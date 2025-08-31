/**
 * @file cloud_thingspeak.cpp
 * @brief Implementation of the ThingSpeak client helpers for ESP32.
 *
 * Uses a plain HTTP/1.1 POST to `api.thingspeak.com/update` with the
 * `application/x-www-form-urlencoded` body. Keeps a single static
 * WiFiClient for lightweight connections.
 */

#include "cloud_thingspeak.h"

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

  #include <WiFi.h>
  #include <WiFiClient.h>
  #include "cloud_config.h"

  /// @brief Shared client used for HTTP transactions.
  static WiFiClient g_tsClient;

  /**
   * @brief Join Wi-Fi if not already connected.
   *
   * Attempts to connect for up to ~15 seconds. The function returns
   * immediately if the ESP32 is already connected.
   *
   * @return true if connected to Wi-Fi, false otherwise.
   */
  bool ts_connectWifi() {
    if (WiFi.status() == WL_CONNECTED) return true;

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 15000UL) {
      delay(200); // small wait between polls
    }
    return WiFi.status() == WL_CONNECTED;
  }

  /**
   * @brief POST a single update to ThingSpeak.
   *
   * Builds a small URL-encoded form body:
   * `api_key=...&field1=...&field2=...&...`
   * and sends it to `/update`. The server replies with a standard
   * HTTP status line which we parse to fill @p httpCode.
   *
   * Field mapping:
   *  - field1: Temperature (°C)
   *  - field2: Humidity (%RH)
   *  - field3: Pressure (hPa)
   *  - field4: Light (raw)
   *  - field5: Sound RMS
   *  - field6: LED state (1=ON, 0=OFF)
   *
   * @param tC        Temperature in Celsius.
   * @param rh        Relative humidity in percent.
   * @param pHpa      Pressure in hectopascals.
   * @param ldr       Light sensor raw ADC reading.
   * @param rms       Sound RMS value (unitless).
   * @param ledState  LED state to report (true=ON → 1, false=OFF → 0).
   * @param httpCode  [out] Parsed HTTP status code (200 = OK).
   * @return true if the HTTP status is 200 OK, false on any error.
   */
  bool ts_post(float tC, float rh, float pHpa, int ldr, float rms, bool ledState, int &httpCode) {
    httpCode = -1;

    // Ensure Wi-Fi connectivity
    if (!ts_connectWifi()) return false;

    // Open TCP connection
    if (!g_tsClient.connect(TS_HOST, TS_PORT)) {
      return false;
    }

    // Build URL-encoded body (ThingSpeak expects '.' decimal separator)
    char body[224];
    int n = snprintf(
      body, sizeof(body),
      "api_key=%s&field1=%.2f&field2=%.2f&field3=%.2f&field4=%d&field5=%.0f&field6=%d",
      TS_WRITE_KEY, tC, rh, pHpa, ldr, rms, (ledState ? 1 : 0)
    );
    if (n <= 0 || n >= (int)sizeof(body)) {
      g_tsClient.stop();
      return false; // body truncated or formatting error
    }

    // Send minimal HTTP/1.1 request
    g_tsClient.print(
      "POST /update HTTP/1.1\r\n"
      "Host: " TS_HOST "\r\n"
      "Connection: close\r\n"
      "Content-Type: application/x-www-form-urlencoded\r\n"
    );
    g_tsClient.print("Content-Length: ");
    g_tsClient.println(strlen(body));
    g_tsClient.print("\r\n");       // end of headers
    g_tsClient.print(body);         // body

    // Read the status line: "HTTP/1.1 200 OK"
    String status = g_tsClient.readStringUntil('\n');
    int sp = status.indexOf(' ');
    if (sp > 0 && status.length() >= sp + 4) {
      httpCode = status.substring(sp + 1, sp + 4).toInt();
    }

    // Drain remainder quickly (don’t block too long)
    unsigned long t1 = millis();
    while (g_tsClient.connected() && (millis() - t1) < 500UL) {
      while (g_tsClient.available()) (void)g_tsClient.read();
    }
    g_tsClient.stop();

    return (httpCode == 200);
  }

#else
  // Compiled out on non-ESP32 targets.
#endif