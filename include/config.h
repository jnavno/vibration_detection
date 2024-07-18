#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define INTERRUPT_PIN GPIO_NUM_7
#define LED_PIN 17
#define INSPECTION_DURATION_SECONDS 11
#define MAX_SAMPLES_PER_CYCLE 100
#define CSV_FILENAME "/accelerometer_data.csv"
#define THRESHOLD_TIER1 210
#define THRESHOLD_TIER2 225

#endif // CONFIG_H
