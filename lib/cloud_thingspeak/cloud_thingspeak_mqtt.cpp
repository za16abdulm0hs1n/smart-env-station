/****
 * @file cloud_thingspeak_mqtt.cpp
 * @brief Thin MQTT publisher for ThingSpeak (ESP32).
 *
 * @details
 * This module exposes a single helper:
 *   - ts_mqtt_publish(tC, rh, pHpa, ldrPct, rms, ledOn)
 *
 * It connects to Wi‑Fi (STA), ensures an MQTT session to the ThingSpeak broker,
 * and publishes a single multi‑field update using the topic:
 *
 *   channels/<TS_CHANNEL_ID>/publish/<TS_WRITE_KEY>
 *
 * The payload is URL‑encoded (key=value&key=value...) as required by ThingSpeak:
 *   - field1: Temperature (°C)
 *   - field2: Humidity (%RH)
 *   - field3: Pressure (hPa)
 *   - field4: Light (% 0..100)
 *   - field5: Sound RMS (unitless)
 *   - field6: LED state (1=ON, 0=OFF)
 *
 * @note All credentials and constants are provided by @ref cloud_config.h.
 * @warning Respect ThingSpeak free‑tier rate limits (≥ 15 s). Your scheduler
 *          should enforce this (e.g., TS_MIN_PERIOD_MS in main.cpp).
 */

#include "cloud_thingspeak_mqtt.h"

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <PubSubClient.h>
  #include <cloud_config.h>

  /// @brief Reusable TCP socket and MQTT client.
  static WiFiClient   wifiClient;   ///< Underlying TCP client
  static PubSubClient mqtt(wifiClient); ///< PubSubClient wrapper

  /**
   * @brief Ensure the ESP32 is connected to Wi‑Fi (station mode).
   *
   * @details
   * - Uses credentials from @ref cloud_config.h (WIFI_SSID / WIFI_PASSWORD).
   * - Attempts connection for ~15–20 s then gives up.
   * - Idempotent: returns immediately if already connected.
   *
   * @return true if Wi‑Fi is connected (WL_CONNECTED), false otherwise.
   */
  static bool ensureWifi() {
    if (WiFi.status() == WL_CONNECTED) return true;

    WiFi.persistent(false);     // do not write credentials to flash
    WiFi.mode(WIFI_STA);        // station mode only
    WiFi.setSleep(false);       // keep radio awake for lower latency
#ifdef WIFI_COUNTRY
    // Note: Some Arduino-ESP32 core versions do not expose WiFi.setCountry().
    // If you need country-specific channels, configure it in your router or
    // upgrade the core/IDF. We skip setting it here for broad compatibility.
    (void)WIFI_COUNTRY;
#endif
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 20000UL) {
      delay(250);
    }
    return WiFi.status() == WL_CONNECTED;
  }

  /**
   * @brief Ensure an MQTT connection to ThingSpeak.
   *
   * @details
   * - Uses host/port from @ref cloud_config.h (MQTT_HOST / MQTT_PORT).
   * - Authenticates with MQTT_CLIENT_ID / MQTT_USERNAME / MQTT_PASSWORD.
   * - Idempotent: returns immediately if already connected.
   *
   * @return true if connected to the broker, false otherwise.
   */
  static bool ensureMqtt() {
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    if (mqtt.connected()) return true;
    // Clean session; no will message
    return mqtt.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
  }

  /**
   * @brief Publish one multi‑field update to ThingSpeak over MQTT.
   *
   * @param tC     Temperature in Celsius.
   * @param rh     Relative humidity in percent.
   * @param pHpa   Pressure in hectopascals.
   * @param ldrPct Light level in percent (0..100).
   * @param rms    Sound RMS (unitless; driver‑specific scale).
   * @param ledOn  LED state to report (true→1, false→0).
   *
   * @return true if @c publish() queued the message to the client, false otherwise.
   *
   * @note A return value of true means the payload was accepted by the MQTT client
   *       library and queued for sending; it does not guarantee the broker stored
   *       the sample. For end‑to‑end confirmation, observe your channel feed.
   */
  bool ts_mqtt_publish(float tC, float rh, float pHpa, int ldrPct, float rms, bool ledOn) {
    if (!ensureWifi()) return false;
    if (!ensureMqtt()) return false;

    // Topic: channels/<CHANNEL_ID>/publish/<WRITE_API_KEY>
    String topic = String("channels/") + TS_CHANNEL_ID + "/publish/" + TS_WRITE_KEY;

    // URL‑encoded body with fixed‑point formatting where useful.
    String payload;
    payload.reserve(160);
    payload  = "field1=" + String(tC, 2);
    payload += "&field2=" + String(rh, 2);
    payload += "&field3=" + String(pHpa, 2);
    payload += "&field4=" + String(ldrPct);
    payload += "&field5=" + String(rms, 0);
    payload += "&field6=" + String(ledOn ? 1 : 0);

    const bool ok = mqtt.publish(topic.c_str(), payload.c_str(), false); // QoS0, retain=false
    mqtt.loop(); // allow background network processing
    return ok;
  }
#endif