#pragma once
#include <Arduino.h>

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
/**
 * @brief Publish one ThingSpeak update over MQTT.
 *
 * Fields:
 *  - field1: Temperature (°C)
 *  - field2: Humidity (%RH)
 *  - field3: Pressure (hPa)
 *  - field4: Light (%)
 *  - field5: Sound RMS
 *  - field6: LED state (0/1)
 */
bool ts_mqtt_publish(float tC, float rh, float pHpa, int ldrPct, float rms, bool ledOn);
#endif