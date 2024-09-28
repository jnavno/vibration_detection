#pragma once

// Pin Definitions
#define LED_PIN 3
#define ACCEL_PWR_PIN 5
#define INTERRUPT_PIN GPIO_NUM_7
#define STATUS_LED_PIN 4

// Timing Definitions
#define SAMPLES 128
#define PHASE_DURATION 30000 // 30 seconds for each phase
#define CYCLES_FOR_5_MIN 10  // 5 minutes = 10 cycles of 30 seconds
#define BLOCK_SIZE 32
#define SAMPLE_RATE 50
#define MAX_SAMPLES (SAMPLE_RATE * (PHASE_DURATION / 1000))
#define FILENAME_FORMAT "/data_phase_%d.csv"
#define PRE_TOGGLE_DELAY 1500  // Delay before toggling accelerometer power
#define MAX_SPIFFS_USAGE 30000 // Max limit of used space in bytes (30KB)\