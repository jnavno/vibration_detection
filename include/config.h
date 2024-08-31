#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Define board-specific settings
#if defined(HELTEC_V3_DEVKIT)
    #define INTERRUPT_PIN GPIO_NUM_7
    #define LED_PIN GPIO_NUM_3
    #define ACCEL_POWER_PIN GPIO_NUM_5
    #define SDA_PIN 41 //GPIO_NUM_40 = 41,
    #define SCL_PIN 42 //GPIO_NUM_40 = 42,
#elif defined(ESP32_S3_DEVKIT)
    #define INTERRUPT_PIN GPIO_NUM_7
    #define LED_PIN 17
    #define ACCEL_POWER_PIN 4
    #define SDA_PIN 10
    #define SCL_PIN 11
#else
    #error "Board not defined. Please define either ESP32_S3_DEVKIT or HELTEC_V3_DEVKIT."
#endif

// Full device weight (in kg), including casing.
#define DEVICE_WEIGHT 0.102

#define INSPECTION_DURATION_SECONDS 11
#define MAX_SAMPLES_PER_CYCLE 100
#define CSV_FILENAME "/accelerometer_data.csv"
#define THRESHOLD_TIER1 210
#define THRESHOLD_TIER2 225

#endif // CONFIG_H