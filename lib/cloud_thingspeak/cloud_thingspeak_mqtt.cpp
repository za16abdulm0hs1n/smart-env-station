/****
 * @file cloud_thingspeak_mqtt.cpp
 * @brief Thin, robust MQTT publisher for ThingSpeak (ESP32).
 *
 * @details
 * Public API (declared in cloud_thingspeak_mqtt.h):
 *   - void ts_mqtt_loop();   // service client, must be called every loop()
 *   - bool ts_mqtt_publish(tC, rh, pHpa, ldrPct, rms, ledOn);
 *
 * Behavior:
 *   - Ensures Wi‑Fi STA connection (best‑effort, ~20 s).
 *   - Ensures MQTT session to mqtt3.thingspeak.com (QoS0, clean session).
 *   - Subscribes to: channels/<TS_CHANNEL_ID>/publish_status
 *     → The broker replies here with a non‑zero entry number on success.
 *   - Publishes multi‑field update on:
 *       channels/<TS_CHANNEL_ID>/publish/<TS_WRITE_KEY>
 *     with URL‑encoded form payload: field1=..&field2=..&...
 *
 * Notes:
 *   - Keep your cadence ≥ 15 s (free tier). Gate in main using TS_MIN_PERIOD_MS.
 *   - Call ts_mqtt_loop() every iteration to keep the session alive and
 *     to receive the publish_status confirmation.
 *   - All credentials/constants come from cloud_config.h.
 */

#include "cloud_thingspeak_mqtt.h"

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <PubSubClient.h>
  #include <Arduino.h>
  #include <cloud_config.h>

  // ------------------------ Globals ------------------------
  static WiFiClient   wifiClient;            ///< Underlying TCP socket
  static PubSubClient mqtt(wifiClient);      ///< MQTT client (QoS0)

  // One‑time setup flags
  static bool g_cb_set       = false;  ///< callback installed?
  static bool g_cfg_done     = false;  ///< buffer/keepalive/socket tuned?
  static bool g_wifi_logged  = false;  ///< printed IP banner already?
  static bool g_subscribed   = false;  ///< subscribed to publish_status?

  // Runtime RX tracking for publish_status
  static volatile bool g_rx_seen = false;    ///< saw any inbound since last publish
  static unsigned long g_rx_last = 0;        ///< last inbound timestamp (ms)

  // ------------------------ Helpers ------------------------
  /**
   * @brief Service the MQTT client.
   *
   * @details
   * Must be called frequently from the main loop to:
   *  - keep the connection alive (pings)
   *  - process incoming packets (e.g., publish_status)
   *  - trigger reconnect logic inside PubSubClient when needed
   */
  void ts_mqtt_loop() { mqtt.loop(); }

  /**
   * @brief Ensure the ESP32 is connected to Wi‑Fi (STA).
   *
   * Uses WIFI_SSID / WIFI_PASSWORD from @ref cloud_config.h.
   * Tries for up to ~20 s. Idempotent: returns immediately when connected.
   *
   * @return true on WL_CONNECTED, false otherwise.
   */
  static bool ensureWifi() {
    if (WiFi.status() == WL_CONNECTED) {
      if (!g_wifi_logged) {
        Serial.print(F("WiFi IP: ")); Serial.println(WiFi.localIP());
        g_wifi_logged = true;
      }
      return true;
    }

    WiFi.persistent(false);   // avoid flash writes
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);     // keep radio awake for low latency
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 20000UL) {
      delay(250);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print(F("WiFi IP: ")); Serial.println(WiFi.localIP());
      g_wifi_logged = true;
    } else {
      Serial.println(F("WiFi not connected (will retry on next publish)."));
    }
    return WiFi.status() == WL_CONNECTED;
  }

  /**
   * @brief Ensure an MQTT connection to ThingSpeak.
   *
   * - Broker: MQTT_HOST:MQTT_PORT (mqtt3.thingspeak.com:1883)
   * - Auth : MQTT_CLIENT_ID / MQTT_USERNAME / MQTT_PASSWORD
   * - QoS0, clean session (PubSubClient default). No Last Will.
   * - Subscribes to publish_status to observe server acceptance.
   *
   * @return true if connected to broker, false otherwise.
   */
  static bool ensureMqtt() {
    mqtt.setServer(MQTT_HOST, MQTT_PORT);     // endpoint

    // One‑time client tuning (safe defaults for ThingSpeak)
    if (!g_cfg_done) {
      mqtt.setBufferSize(256);   // topic+payload comfortably fit
      mqtt.setKeepAlive(60);     // seconds
      mqtt.setSocketTimeout(8);  // seconds
      g_cfg_done = true;
    }

    // One‑time inbound logger / flagger
    if (!g_cb_set) {
      mqtt.setCallback([](char* topic, byte* payload, unsigned int len) {
        Serial.print(F("[MQTT] RX ")); Serial.print(topic); Serial.print(F(": "));
        for (unsigned int i = 0; i < len; ++i) Serial.write(payload[i]);
        Serial.println();
        g_rx_seen = true;
        g_rx_last = millis();
      });
      g_cb_set = true;
    }

    // Already connected?
    if (mqtt.connected()) return true;

    Serial.print(F("[MQTT] connecting... "));
    const bool ok = mqtt.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
    if (!ok) {
      Serial.print(F("FAIL state=")); Serial.println(mqtt.state());
      return false;
    }
    Serial.println(F("OK"));

    // Subscribe to ThingSpeak publish_status
    const String statusTopic = String("channels/") + TS_CHANNEL_ID + "/subscribe";
    if (mqtt.subscribe(statusTopic.c_str())) {
      Serial.print(F("[MQTT] subscribed: ")); Serial.println(statusTopic);
      g_subscribed = true;
    } else {
      Serial.print(F("[MQTT] subscribe FAILED: ")); Serial.println(statusTopic);
      g_subscribed = false;
    }
    return true;
  }

  // ------------------------ Publish ------------------------
  /**
   * @brief Publish one multi‑field update to ThingSpeak over MQTT.
   *
   * @param tC     Temperature in Celsius.
   * @param rh     Relative humidity in percent.
   * @param pHpa   Pressure in hectopascals.
   * @param ldrPct Light in percent (0..100).
   * @param rms    Sound RMS (unitless, driver‑specific).
   * @param ledOn  LED state (true→1, false→0).
   *
   * @return true if payload was queued to the MQTT client;
   *         false if Wi‑Fi/MQTT unavailable or publish failed.
   *
   * @note A return of true does not guarantee server storage—watch the
   *       publish_status callback for a non‑zero entry number to confirm.
   */
  bool ts_mqtt_publish(float tC, float rh, float pHpa, int ldrPct, float rms, bool ledOn) {
    if (!ensureWifi()) return false;
    if (!ensureMqtt()) return false;

    // Topic: channels/<CHANNEL_ID>/publish/<WRITE_KEY>
    const String topic = String("channels/") + TS_CHANNEL_ID + "/publish";;

    // Keep payload compact; PubSubClient default buffer is small.
    String payload;
    payload.reserve(160);
    payload  =  F("field1="); payload += String(tC, 2);
    payload += F("&field2="); payload += String(rh, 2);
    payload += F("&field3="); payload += String(pHpa, 2);
    payload += F("&field4="); payload += String(ldrPct);
    payload += F("&field5="); payload += String(rms, 0);
    payload += F("&field6="); payload += (ledOn ? F("1") : F("0"));

    Serial.print(F("[DBG] topic: "));   Serial.println(topic);
    Serial.print(F("[DBG] payload(")); Serial.print(payload.length());
    Serial.print(F("): "));            Serial.println(payload);

    // Reset ack tracking and publish (QoS0, retain=false)
    g_rx_seen = false;
    const bool ok = mqtt.publish(topic.c_str(), payload.c_str(), /*retain*/false);
    if (!ok) {
      Serial.println(F("[MQTT] publish() returned false"));
      return false;
    }
    Serial.println(F("MQTT publish sent"));

    // Give the client a short window to receive publish_status now.
    const unsigned long t0 = millis();

    return true;
  }

#endif  // ESP32