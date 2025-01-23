#pragma once

// Pin Definitions
#define LED_PIN GPIO_NUM_3
#define VEXT_CTRL_PIN GPIO_NUM_36
#define INTERRUPT_PIN GPIO_NUM_7
#define STATUS_LED_PIN GPIO_NUM_4

// Timing Definitions
#define PHASE_DURATION 10000        // 10 seconds per phase
#define MAX_CYCLES 3          // Limit to 3 cycles
#define SAMPLE_RATE 1000              // 50 samples per second
#define BLOCK_SIZE 32               // Block size for FIFO buffer
#define MAX_SAMPLES (SAMPLE_RATE * (PHASE_DURATION / 1000)) // Max samples per phase

// File and SPIFFS Settings
#define FILENAME_FORMAT "/data_phase_%d.csv"
#define MAX_SPIFFS_USAGE 30000 // Max SPIFFS space (30KB)

// Delays
#define PRE_TOGGLE_DELAY 100 // Delay before toggling accelerometer power
