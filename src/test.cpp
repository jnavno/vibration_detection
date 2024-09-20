#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "SPIFFS.h"
#include <Wire.h>

#define LED_PIN 3                    // Define the LED pin (GPIO3)
#define ACCEL_PWR_PIN 5              // Define the pin to control the MOSFET
#define INTERRUPT_PIN GPIO_NUM_7     // GPIO7 for interrupt from the shake sensor
#define QUICK_BLINK_DELAY 100        // Quick blink delay in milliseconds
#define SLOW_BLINK_DELAY 1000        // Slow blink delay in milliseconds
#define DATA_STREAM_DURATION 2000    // Duration for data streaming in milliseconds
#define NO_DATA_STREAM_DURATION 1000 // Duration for no data streaming in milliseconds
#define PRE_TOGGLE_DELAY 1000        // Increased delay before toggling accel power
#define INIT_DELAY 500               // Delay after powering on the accelerometer
#define CYCLE_COUNT 3                // Number of cycles for reading data

RTC_DATA_ATTR int cycleCount = 0;    // Keep track of completed cycles during reboots
RTC_DATA_ATTR int bootCount = 0;     // Keep track of the boot count (deep sleep)
Adafruit_MPU6050 mpu;
volatile bool wakeup_flag = false;   // Flag to track wake-up by interrupt

// Function prototypes
void blinkLED(int delayTime);
void readAccelerometer();
void toggleAccelPower(bool state);
bool initializeMPU();
void IRAM_ATTR handleWakeUpInterrupt();
void enterDeepSleep();
void powerCycleMPU();

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);  // Set LED pin as output
    pinMode(ACCEL_PWR_PIN, OUTPUT);
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);  // Set interrupt pin as input
    delay(2000);

    // Increment boot count for each wake-up
    bootCount++;
    Serial.print("Boot number: ");
    Serial.println(bootCount);

    // Power on the accelerometer
    toggleAccelPower(true);
    delay(PRE_TOGGLE_DELAY);  // Allow time for accelerometer to power up

    // Power cycle the MPU6050 after deep sleep
    powerCycleMPU();

    // Initialize MPU6050 if it's the first boot or after a reset
    if (!initializeMPU()) {
        Serial.println("Failed to initialize MPU6050 after power cycle.");
        while (1);
    }
    Serial.println("MPU6050 detected and initialized!");

    // Perform 3 cycles of reading before going into deep sleep
    for (int i = 0; i < CYCLE_COUNT; i++) {
        Serial.print("Starting cycle ");
        Serial.println(i + 1);

        // Reinitialize the MPU6050 before starting the reading stream
        if (!initializeMPU()) {
            Serial.println("Reinitialization of MPU6050 failed.");
            while (1);
        }
        Serial.println("MPU6050 reinitialized for data stream.");

        // Stream accelerometer data for the defined duration
        unsigned long streamStartTime = millis();
        while (millis() - streamStartTime < DATA_STREAM_DURATION) {
            blinkLED(QUICK_BLINK_DELAY);  // Blink LED quickly
            readAccelerometer();
        }

        // No data stream period
        unsigned long noStreamStartTime = millis();
        while (millis() - noStreamStartTime < NO_DATA_STREAM_DURATION) {
            blinkLED(SLOW_BLINK_DELAY);  // Blink LED slowly
            delay(SLOW_BLINK_DELAY);     // Keep the LED off for the remaining time
        }

        // Update the cycle count
        cycleCount++;
        Serial.print("Completed cycle count: ");
        Serial.println(cycleCount);

        if (i < CYCLE_COUNT - 1) {
            // Power cycle the accelerometer for the next reading
            toggleAccelPower(false);
            delay(PRE_TOGGLE_DELAY);
            toggleAccelPower(true);
            delay(PRE_TOGGLE_DELAY);

            if (!initializeMPU()) {
                Serial.println("Failed to reinitialize MPU6050 after cycle.");
                while (1);
            }
        }
    }

    // After 3 cycles, enable interrupt for future wakeup and go to deep sleep
    enterDeepSleep();
}

void loop() {
    // Not used
}

// ISR for handling wake-up interrupt
void IRAM_ATTR handleWakeUpInterrupt() {
    wakeup_flag = true;  // Set wakeup flag when interrupt is triggered
}

void blinkLED(int delayTime) {
    digitalWrite(LED_PIN, HIGH);  // Turn LED on
    delay(delayTime);             // Wait for a short period
    digitalWrite(LED_PIN, LOW);   // Turn LED off
    delay(delayTime);             // Wait for a short period
}

void toggleAccelPower(bool state) {
    digitalWrite(ACCEL_PWR_PIN, state ? HIGH : LOW);
}

// New function to handle complete power cycle of MPU6050
void powerCycleMPU() {
    toggleAccelPower(false);  // Turn off MPU6050
    delay(500);               // Wait for the MPU to power down fully
    toggleAccelPower(true);   // Turn MPU6050 back on
    delay(PRE_TOGGLE_DELAY);  // Allow time for the MPU to power back up
}

bool initializeMPU() {
    if (!mpu.begin()) {
        // Add a small delay and retry
        delay(500);
        if (!mpu.begin()) {
            return false;
        }
    }
    delay(INIT_DELAY);  // Allow MPU to settle
    return true;
}

void readAccelerometer() {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
        // Print accelerometer data
        Serial.print("X: ");
        Serial.print(a.acceleration.x);
        Serial.print(" m/s^2 ");
        Serial.print("Y: ");
        Serial.print(a.acceleration.y);
        Serial.print(" m/s^2 ");
        Serial.print("Z: ");
        Serial.print(a.acceleration.z);
        Serial.print(" m/s^2 ");
        Serial.println();
    } else {
        Serial.println("Failed to read from MPU6050");
    }
}

// Enter deep sleep and enable the interrupt for the next wake-up
void enterDeepSleep() {
    Serial.println("Completed all cycles, entering deep sleep...");

    // Enable interrupt for future wake-up
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleWakeUpInterrupt, RISING);
    esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, 1);  // Enable wakeup on GPIO7 rising edge

    // Enter deep sleep
    esp_deep_sleep_start();
}
