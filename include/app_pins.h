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
constexpr uint8_t PIN_LDR = 32; ///< Light Sensor (Base A3 → ESP32 P32)

constexpr int     PIN_SDA = 21; ///< I²C data (SDA)
constexpr int     PIN_SCL = 22; ///< I²C clock (SCL)

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