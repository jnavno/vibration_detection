#pragma once

// Define board-specific settings for ESP32_S3_DEVKIT
#define INTERRUPT_PIN GPIO_NUM_7
#define LED_PIN 17
#define ACCEL_POWER_PIN GPIO_NUM_4
#define SDA_PIN 10  // I2C SDA Pin
#define SCL_PIN 11  // I2C SCL Pin

// Device specifications
#define DEVICE_WEIGHT 0.102  // Full device weight in kg
#define INSPECTION_DURATION_SECONDS 11
#define MAX_SAMPLES_PER_CYCLE 100
#define CSV_FILENAME "/accelerometer_data.csv"

// Thresholds for accelerometer data
#define THRESHOLD_TIER1 210
#define THRESHOLD_TIER2 225
