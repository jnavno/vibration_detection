#pragma once

// Pin Definitions
#define LED_PIN GPIO_NUM_3
#define VEXT_CTRL_PIN GPIO_NUM_36
#define INTERRUPT_PIN GPIO_NUM_7
#define STATUS_LED_PIN GPIO_NUM_4
#define ALERT_LED_PIN GPIO_NUM_5
#define SDA_PIN GPIO_NUM_41
#define SCL_PIN GPIO_NUM_42
#define SD_CS_PIN GPIO_NUM_19
#define SD_MOSI_PIN GPIO_NUM_38
#define SD_MISO_PIN GPIO_NUM_39
#define SD_SCK_PIN GPIO_NUM_37

//  Battery Thresholds
#define MIN_BATT_VOLTAGE 3.3 //low voltage alert threshold
#define MAX_BATT_VOLTAGE 4.2 //max voltage alert threshold
#define SOC_THRESHOLD 20    //SOC alert threshold (e.g., 20% battery SOC)

// Adaptive Sampling Configuration
extern int SAMPLE_RATE;  // Sampling rate dynamically adjusted
extern int SAMPLES;  // Dynamically updated sample size
#define DEFAULT_SAMPLE_RATE 20  // Low-power mode default

// Sampling Duration & Buffer Size
// ==== ACCELEROMETER SAMPLING CONFIG ====
#ifndef ACCEL_SAMPLE_RATE_HZ
  #define ACCEL_SAMPLE_RATE_HZ 333
#endif

#ifndef ACCEL_DURATION_SECONDS
  #define ACCEL_DURATION_SECONDS 12
#endif

#define ACCEL_NUM_SAMPLES (ACCEL_SAMPLE_RATE_HZ * ACCEL_DURATION_SECONDS)

#ifndef NUM_BLOCKS
  #define NUM_BLOCKS 25  // Total blocks per session (12s each)
#endif
// ~5 minutes active logging per session: NUM_BLOCKS × ACCEL_DURATION_SECONDS

// Filepaths
#define SPIFFS_FILE "/data.csv"
#define CLASSIFICATION_FILE "/classification_log.txt"
#define STATUS_FILE "/status_log.txt"
#define STATE_FILENAME "/log_state.txt"

// Vibration Detection Thresholds and Frequencies
#ifndef MPU6050_DEFAULT_ADDRESS
  #define MPU6050_DEFAULT_ADDRESS 0x68
#endif
#define SAW_MIN_FREQ 30.0
#define SAW_MAX_FREQ 60.0
#define CHAINSAW_MIN_FREQ 200.0
#define CHAINSAW_MAX_FREQ 300.0
#define MACHETE_MIN_FREQ 5.0
#define MACHETE_MAX_FREQ 10.0

// SPIFFS Settings
#define SPIFFS_FILE "/data.csv"  // Single data storage file

// Delays
#define PRE_TOGGLE_DELAY 100  // Delay before toggling accelerometer power
#define ENABLE_INTERRUPT true
#define MONITOR_DURATION 60000  // 60 seconds
#define VIBRATION_THRESHOLD 0.5  // Threshold for increasing sampling rate

// SD logger specific
#ifdef ENABLE_SD_LOGGER
  #define BLOCKS_PER_BATCH 25 // ~5 minutes worth of blocks
  #define PASSIVE_MODE false  // true: log for 24h (5min logging + 20min sleep loops), false: log once for 5min then sleep permanently
  #define BATCH_SLEEP_MINUTES 20.0  // Sleep 20 minutes between batches
  #define TOTAL_LOGGING_HOURS 24.0  // Full 24 hours of logging
  #define SIMULATED_BATCH_SECONDS 1200  // 20 min sleep + ~5 min active = ~25 min total
  #define BATCH_DURATION_MS (BLOCKS_PER_BATCH * ACCEL_DURATION_SECONDS * 1000 + 200 * BLOCKS_PER_BATCH)
#endif