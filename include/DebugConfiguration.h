#pragma once

#include <Arduino.h>

// ==============================
// 🔹 DEBUG CONFIGURATION
// ==============================

// Define DEBUG levels (0 = off, 1 = minimal, 2 = full)
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 1  // Default to minimal debugging
#endif

#if DEBUG_LEVEL > 0
    #define INIT_DEBUG_SERIAL() Serial.begin(115200)  // Use Serial if debugging is enabled
    #define LOG_DEBUG(...) Serial.printf(__VA_ARGS__)
    #define LOG_DEBUGLN(...) Serial.println(__VA_ARGS__)
    #define DUMP_SD_PINS() do { \
        Serial.printf("CS: %d, MOSI: %d, MISO: %d, SCK: %d\n", \
            digitalRead(SD_CS_PIN), digitalRead(SD_MOSI_PIN), \
            digitalRead(SD_MISO_PIN), digitalRead(SD_SCK_PIN)); \
    } while(0)
#else
    #define INIT_DEBUG_SERIAL()  // Do nothing if debugging is off
    #define LOG_DEBUG(...)  // No-op when debugging is off
    #define LOG_DEBUGLN(...)
#endif

// ==============================
// 🔹 FEATURE TOGGLES
// ==============================

// Enable Power-Saving Mode (use `-DPOWER_SAVE_MODE` in platformio.ini)
#ifdef POWER_SAVE_MODE
    #define DISABLE_UART 1
    #define DISABLE_WIFI 1
    #define DISABLE_BLUETOOTH 1
#else
    #define DISABLE_UART 0
    #define DISABLE_WIFI 0
    #define DISABLE_BLUETOOTH 0
#endif

// ==============================
// 🔹 FEATURE TOGGLE LOGGING
// ==============================

#if DEBUG_LEVEL > 1
    #define LOG_FEATURE_TOGGLE() do { \
        LOG_DEBUGLN("Feature Toggles Active:"); \
        LOG_DEBUG(" - DISABLE_UART: "); LOG_DEBUGLN(DISABLE_UART); \
        LOG_DEBUG(" - DISABLE_WIFI: "); LOG_DEBUGLN(DISABLE_WIFI); \
        LOG_DEBUG(" - DISABLE_BLUETOOTH: "); LOG_DEBUGLN(DISABLE_BLUETOOTH); \
    } while (0)
#else
    #define LOG_FEATURE_TOGGLE()  // No-op when debugging is off
#endif