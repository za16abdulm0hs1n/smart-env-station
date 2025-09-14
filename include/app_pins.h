/**
 * @file app_pins.h
 * @brief Centralized pin mapping for Arduino Sensor Kit Base.
 *
 * This header abstracts the differences between Arduino Uno and ESP32-S boards.
 * When compiled for each target, the correct mapping is selected automatically.
 */

#pragma once
#include <Arduino.h>

#if defined(Esp32) || defined(ARDUINO_ARCH_ESP32)
/**
 * @section ESP32 NodeMCU-32S Mapping
 *
 * Pin assignments for ESP32S (NodeMCU-32S dev board), wired to Arduino Sensor Kit Base.
 */
constexpr uint8_t PIN_BTN = 25; ///< Button (Base D4 → ESP32 P25)
constexpr uint8_t PIN_LED = 26; ///< LED (Base D6 → ESP32 P26)
constexpr uint8_t PIN_BUZ = 27; ///< Buzzer (Base D5 → ESP32 P27)

constexpr uint8_t PIN_POT = 34; ///< Potentiometer (Base A0 → ESP32 P34, ADC-only)
constexpr uint8_t PIN_SND = 35; ///< Sound Sensor (Base A1 → ESP32 P35, ADC-only)
constexpr uint8_t PIN_LDR = 39; ///< Light Sensor (Base A3 → ESP32 P39, ADC-only)

constexpr int     PIN_SDA = 21; ///< I²C data (SDA)
constexpr int     PIN_SCL = 22; ///< I²C clock (SCL)

/**
 * @section Status LED Pin Assignments
 *
 * Pin assignments for status LEDs (5x red for sensors, 1x blue for TX, 1x yellow for error, 1x green for power/heartbeat).
 */
constexpr uint8_t PIN_LED_SENSOR1 = 13;   ///< Red LED 1 (Sensor 1 status)
constexpr uint8_t PIN_LED_SENSOR2 = 14;   ///< Red LED 2 (Sensor 2 status)
constexpr uint8_t PIN_LED_SENSOR3 = 16;  ///< Red LED 3 (Sensor 3 status)
constexpr uint8_t PIN_LED_SENSOR4 = 17;  ///< Red LED 4 (Sensor 4 status)
constexpr uint8_t PIN_LED_SENSOR5 = 33;   ///< Red LED 5 (Sensor 5 status)
constexpr uint8_t PIN_LED_TX      = 18;  ///< Blue LED (TX activity)
constexpr uint8_t PIN_LED_ERROR   = 19;  ///< Yellow LED (Error indicator)
constexpr uint8_t PIN_LED_PWR     = 15;  ///< Green LED (Power/Heartbeat)


#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_ARCH_AVR)
/**
 * @section Arduino Uno Mapping
 *
 * Pin assignments for Arduino Uno (ATmega328P), wired to Arduino Sensor Kit Base.
 */
constexpr uint8_t PIN_BTN = 4;   ///< Button (Base D4 → Uno D4)
constexpr uint8_t PIN_LED = 6;   ///< LED (Base D6 → Uno D6)
constexpr uint8_t PIN_BUZ = 5;   ///< Buzzer (Base D5 → Uno D5)

constexpr uint8_t PIN_POT = A0;  ///< Potentiometer (Base A0 → Uno A0)
constexpr uint8_t PIN_SND = A2;  ///< Sound Sensor (Base A1 → Uno A1)
constexpr uint8_t PIN_LDR = A3;  ///< Light Sensor (Base A2 → Uno A2)

// I²C on Uno: SDA = A4, SCL = A5 (no explicit defines required)

#else
#  error "Unsupported board. Please extend app_pins.h for your target."
#endif

