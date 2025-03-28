#pragma once

// Pin Definitions
#define LED_PIN GPIO_NUM_3
#define VEXT_CTRL_PIN GPIO_NUM_36
#define INTERRUPT_PIN GPIO_NUM_7
#define STATUS_LED_PIN GPIO_NUM_4
#define ALERT_LED_PIN GPIO_NUM_5       // Pin to read ALRT signal from MAX17048
#define SDA_PIN GPIO_NUM_41
#define SCL_PIN GPIO_NUM_42

//  Battery Thresholds
#define MIN_BATT_VOLTAGE 3.3 //low voltage alert threshold
#define MAX_BATT_VOLTAGE 4.2 //max voltage alert threshold
#define SOC_THRESHOLD 20    //SOC alert threshold (e.g., 20% battery SOC)

// Adaptive Sampling Configuration
extern int SAMPLE_RATE;  // Sampling rate dynamically adjusted
extern int SAMPLES;  // Dynamically updated sample size
#define DEFAULT_SAMPLE_RATE 20  // Low-power mode default

// Sampling Duration & Buffer Size
#define SAMPLING_DURATION 60  // 60 seconds of monitoring
#define SAMPLING_DURATION_ACCEL 12  // time block readings in secods
#define SAMPLE_RATE_ACCEL_HZ 1000   //
#define MAX_BUFFER_SIZE 24000  // Maximum buffer allocation (400 Hz * 60 sec)
#define BLOCK_SIZE 128  // Define BLOCK_SIZE for FIFO buffer handling

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
