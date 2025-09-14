#pragma once
#include <Arduino.h>
#include "app_pins.h"

/**
 * @file led_status.h
 * @brief Status panel for the sensor board: 5× sensor LEDs (red),
 *        TX (blue), ERROR (yellow), POWER/heartbeat (green).
 *
 * Usage:
 *  - Call ledsBegin() in setup().
 *  - Call ledsUpdate() every ~50–100 ms.
 *  - Call ledsSetSensorStatus(...) whenever sensor enable/health changes.
 *  - Call ledsSetError(true/false) on error state changes.
 *  - Call ledsPulseTx() right when you send a packet.
 */

// --- Lifecycle ---
void ledsBegin();                 ///< Configure pins and clear all LEDs.
void ledsUpdate();                ///< Run heartbeat + auto-clear TX pulse (non-blocking).

// --- Events / state setters ---
void ledsPulseTx(uint16_t ms=120);///< Short TX flash (blue), duration in ms.
void ledsSetError(bool on);       ///< ERROR (yellow): true=ON, false=OFF.

// 5 sensor LEDs: true = enabled & healthy; false = disabled/stale/error.
void ledsSetSensorStatus(bool tempOk,
                         bool pressOk,
                         bool lightOk,
                         bool soundOk,
                         bool accelOk);