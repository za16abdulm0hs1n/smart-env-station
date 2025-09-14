#pragma once
#include <Arduino.h>

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

/**
 * @file cloud_thingspeak_mqtt.h
 * @brief MQTT interface for publishing sensor data to ThingSpeak.
 *
 * This header declares functions for publishing environmental sensor data
 * such as temperature, humidity, pressure, light intensity, sound level,
 * and LED state to ThingSpeak via MQTT protocol.
 */

/**
 * @brief Processes MQTT client loop to maintain connection and handle messages.
 *
 * This function should be called regularly to keep the MQTT client connected
 * and to process incoming/outgoing MQTT messages.
 */
void ts_mqtt_loop();

/**
 * @brief Publish a single ThingSpeak update over MQTT.
 *
 * Sends sensor readings and LED state to ThingSpeak fields via MQTT.
 *
 * @param tC Temperature in degrees Celsius.
 * @param rh Relative Humidity in percent (%RH).
 * @param pHpa Atmospheric Pressure in hectopascals (hPa).
 * @param ldrPct Light intensity as a percentage (%).
 * @param rms Sound root mean square (RMS) level.
 * @param ledOn LED state, true for ON (1), false for OFF (0).
 * @return true if the publish was successful, false otherwise.
 */
bool ts_mqtt_publish(float tC, float rh, float pHpa, int ldrPct, float rms, bool ledOn);

#endif