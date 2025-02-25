#pragma once

// Serial Baud Rate for debugging
#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200 // Default serial baud rate
#endif

// DebugConfiguration.h
#ifdef CFG_RELEASE
    #define debug_printf(...)  // No operation in release mode
    #define debug_print(...)   // No operation in release mode
    #define debug_println(...) // No operation in release mode
#else
    #define debug_printf(...)   Serial.printf(__VA_ARGS__)
    #define debug_print(...)    Serial.print(__VA_ARGS__)
    #define debug_println(...)  Serial.println(__VA_ARGS__)
#endif