#pragma once
#include <Arduino.h>

/**
 * @file cloud_config.h
 * @brief Wi‑Fi credentials and ThingSpeak configuration for HTTP and MQTT.
 *
 * This header centralizes all user-tunable cloud settings for the project.
 * - HTTP posting uses TS_HOST / TS_PORT / TS_WRITE_KEY.
 * - MQTT publishing (optional) uses the MQTT_* and TS_CHANNEL_ID defines.
 *
 * @warning This file contains private credentials. Do NOT commit it to
 *          public repositories.
 */

// ============================================================================
// Wi‑Fi (2.4 GHz only for ESP32)
// ============================================================================

/**
 * @def WIFI_SSID
 * @brief Wi‑Fi SSID (network name). ESP32 must join 2.4 GHz (not 5 GHz).
 */
#define WIFI_SSID     "Za16's_2_4"

/**
 * @def WIFI_PASSWORD
 * @brief Wi‑Fi password for the SSID.
 */
#define WIFI_PASSWORD "KrK2!26o!"


/**
 * @def WIFI_COUNTRY
 * @brief 2‑letter ISO country code to enable correct Wi‑Fi channels (e.g., 12/13 in EU).
 *        Used by ts_connectWifi() to call WiFi.setCountry().
 */
#define WIFI_COUNTRY  "DE"


// ============================================================================
// ThingSpeak — HTTP (used by ts_post())
// ============================================================================

/**
 * @def TS_HOST
 * @brief ThingSpeak HTTP server host.
 */
#define TS_HOST       "api.thingspeak.com"

/**
 * @def TS_PORT
 * @brief ThingSpeak HTTP server port (80).
 */
#define TS_PORT       80

/**
 * @def TS_WRITE_KEY
 * @brief ThingSpeak Write API Key (Channel Settings → API Keys → Write).
 *
 * This key authorizes inserts into your channel for HTTP and MQTT publish topic.
 */
#define TS_WRITE_KEY  "KU84WOREJ9QZ10CI"


/**
 * @def TS_MIN_PERIOD_MS
 * @brief Minimum post interval per ThingSpeak free tier (>= 15 s). We use 20 s.
 */
#define TS_MIN_PERIOD_MS  20000UL


// ============================================================================
// ThingSpeak — MQTT (optional, used by ts_mqtt_publish())
// To enable MQTT in code, you can add: #define USE_MQTT 1 (or keep HTTP default).
// ============================================================================

/**
 * @def MQTT_HOST
 * @brief ThingSpeak MQTT broker hostname.
 */
#define MQTT_HOST        "mqtt3.thingspeak.com"

/**
 * @def MQTT_PORT
 * @brief MQTT port (1883 = plaintext, 8883 = TLS). Keep 1883 unless you enable TLS.
 */
#define MQTT_PORT        1883

/**
 * @def TS_CHANNEL_ID
 * @brief Your numeric ThingSpeak channel ID as a string (e.g., "123456").
 *        Required by the MQTT publish topic: channels/&lt;TS_CHANNEL_ID&gt;/publish/&lt;TS_WRITE_KEY&gt;
 *
 * @note Fill this with YOUR channel number from the channel page URL.
 */
#define TS_CHANNEL_ID    "3055031"

/**
 * @def MQTT_CLIENT_ID
 * @brief Client ID from ThingSpeak → Devices → MQTT → (your device) → Download Credentials.
 */
#define MQTT_CLIENT_ID   "LRYXJzo1OTEhKigjHCsnMgg"

/**
 * @def MQTT_USERNAME
 * @brief Username from the downloaded MQTT credentials.
 */
#define MQTT_USERNAME    "LRYXJzo1OTEhKigjHCsnMgg"

/**
 * @def MQTT_PASSWORD
 * @brief Password from the downloaded MQTT credentials.
 */
#define MQTT_PASSWORD    "rJKSgRMJwS3cEL2YthbOG8Ia"


// ============================================================================
// Feature switch (HTTP vs MQTT)
// ============================================================================

/**
 * @def USE_MQTT
 * @brief Set to 1 to publish via MQTT, or 0 to use HTTP posting.
 *        You can also override this in a .cpp (define before including this header).
 */
#ifndef USE_MQTT
  #define USE_MQTT 1
#endif

// (No preprocessor string-index checks; credentials above are taken as-is.)