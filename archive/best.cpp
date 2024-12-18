#include <Arduino.h>
#include "SPIFFS.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Pin Definitions
#define LED_PIN 3                    // Define the LED pin (GPIO3)
#define ACCEL_PWR_PIN 5              // Define the pin to control the MOSFET
#define INTERRUPT_PIN GPIO_NUM_7     // GPIO7 for interrupt from the shake sensor

// Timing and Cycle Definitions
#define QUICK_BLINK_DELAY 100        // Quick blink delay in milliseconds
#define SLOW_BLINK_DELAY 1000        // Slow blink delay in milliseconds
#define INSPECTION_DURATION_SECONDS 11 // Duration to buffer data in FIFO
#define PRE_TOGGLE_DELAY 1500        // Delay before toggling accelerometer power
#define INIT_DELAY 1000              // Delay after powering on the accelerometer
#define CYCLE_COUNT 3                // Number of cycles for reading data

// RTC Data Attributes for Deep Sleep Preservation
RTC_DATA_ATTR int cycleCount = 0;    // Keep track of completed cycles during reboots
RTC_DATA_ATTR int bootCount = 0;     // Keep track of the boot count (deep sleep)

// MPU6050 Object
MPU6050 mpu;
volatile bool wakeup_flag = false;   // Flag to track wake-up by interrupt

// Function Prototypes
void blinkLED(int delayTime);
void toggleAccelPower(bool state);
bool initializeMPU();
void readFIFOData();
void IRAM_ATTR handleWakeUpInterrupt();
void enterDeepSleep();
void powerCycleMPU();
bool evaluateThreshold(float totalWork, int stage);
bool checkFIFOOverflow();

// Constants for Thresholds
#define FIRST_THRESHOLD 650.0
#define SECOND_THRESHOLD 750.0
#define THIRD_THRESHOLD 7500.0

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);  // Set LED pin as output
    pinMode(ACCEL_PWR_PIN, OUTPUT);
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);  // Set interrupt pin as input

    Wire.begin(41, 42);
    Wire.setClock(30000);  // Set I2C to 100kHz for stability
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
        while (1);  // Stop execution if initialization fails
    }
    Serial.println("MPU6050 detected and initialized!");

    // Perform 3 cycles of reading before going into deep sleep
    for (int i = 0; i < CYCLE_COUNT; i++) {
        Serial.print("Starting cycle ");
        Serial.println(i + 1);

        // Reinitialize MPU6050 before starting the reading stream
        if (!initializeMPU()) {
            Serial.println("Reinitialization of MPU6050 failed.");
            while (1);  // Stop execution if reinitialization fails
        }
        Serial.println("MPU6050 reinitialized for data stream.");

        // Read and process data using FIFO for 11 seconds
        readFIFOData();

        // Blink slowly between cycles
        unsigned long noStreamStartTime = millis();
        while (millis() - noStreamStartTime < SLOW_BLINK_DELAY) {
            blinkLED(SLOW_BLINK_DELAY);  // Slow blink indicates idle
        }

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
                while (1);  // Stop execution if reinitialization fails
            }
        }
    }

    // After 3 cycles, enter deep sleep
    enterDeepSleep();
}

void loop() {
    // Not used in deep sleep mode
}

// ISR for handling wake-up interrupt
void IRAM_ATTR handleWakeUpInterrupt() {
    wakeup_flag = true;  // Set wakeup flag when interrupt is triggered
}

void blinkLED(int delayTime) {
    digitalWrite(LED_PIN, HIGH);  // Turn LED on
    delay(delayTime);             // Wait
    digitalWrite(LED_PIN, LOW);   // Turn LED off
    delay(delayTime);             // Wait
}

void toggleAccelPower(bool state) {
    digitalWrite(ACCEL_PWR_PIN, state ? LOW : HIGH);
}

void powerCycleMPU() {
    digitalWrite(ACCEL_PWR_PIN, LOW);  // Turn off MPU6050
    delay(2000);  // Ensure sufficient time for power-down
    digitalWrite(ACCEL_PWR_PIN, HIGH); // Power back on
    delay(2500);  // Allow sufficient time for MPU6050 to stabilize after powering on
}


// Initialize MPU6050 with proper FIFO configuration
bool initializeMPU() {
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        return false;
    }

    // Reset and configure FIFO
    mpu.setFIFOEnabled(false);     // Disable FIFO to configure it
    mpu.resetFIFO();               // Clear FIFO buffer
    mpu.setAccelFIFOEnabled(true); // Enable accelerometer FIFO
    mpu.setFIFOEnabled(true);      // Enable FIFO

    // Set sample rate to 15Hz (66.67ms interval)
    // Set sample rate to around 5Hz (1000 / (199 + 1) = ~5Hz)
    mpu.setRate(199);

    // Optionally set accelerometer range (default is ±2g)
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    delay(INIT_DELAY);  // Allow MPU to settle

    return true;
}

// Function to check for FIFO overflow
bool checkFIFOOverflow() {
    if (mpu.getIntFIFOBufferOverflowStatus()) {
        Serial.println("FIFO overflow detected!");
        return true;
    }
    return false;
}

// Function to read data from FIFO buffer
void readFIFOData() {
    uint8_t fifoBuffer[1024];  // FIFO buffer size
    int16_t accelX, accelY, accelZ;
    float totalWorkX = 0, totalWorkY = 0, totalWorkZ = 0;
    float totalWork = 0;

    // Instead of waiting 11 seconds, check FIFO every second
    for (int i = 0; i < INSPECTION_DURATION_SECONDS; i++) {
        delay(1000);  // Delay for 1 second between reads

        // Check FIFO count
        uint16_t fifoCount = mpu.getFIFOCount();
        if (fifoCount == 0) {
            Serial.println("No data in FIFO.");
            return;
        }

        // Check for FIFO overflow
        if (checkFIFOOverflow()) {
            mpu.resetFIFO();  // Reset FIFO if overflow is detected
            return;
        }

        // Read data from FIFO
        while (fifoCount >= 6) {  // Each accelerometer reading = 6 bytes
            mpu.getFIFOBytes(fifoBuffer, 6);
            fifoCount -= 6;

            accelX = (fifoBuffer[0] << 8) | fifoBuffer[1];
            accelY = (fifoBuffer[2] << 8) | fifoBuffer[3];
            accelZ = (fifoBuffer[4] << 8) | fifoBuffer[5];

            // Convert to g-force and accumulate work
            totalWorkX += fabs(accelX / 16384.0);
            totalWorkY += fabs(accelY / 16384.0);
            totalWorkZ += fabs(accelZ / 16384.0);
        }
    }

    // Compute total work
    totalWork = totalWorkX + totalWorkY + totalWorkZ;
    Serial.print("Total Work: ");
    Serial.println(totalWork);

    // Evaluate thresholds and make decisions
    for (int stage = 1; stage <= 3; stage++) {
        if (evaluateThreshold(totalWork, stage)) {
            Serial.print("Stage ");
            Serial.print(stage);
            Serial.println(": Threshold exceeded, continuing...");
        } else {
            Serial.println("Work below threshold, going to sleep...");
            enterDeepSleep();  // Go to sleep if threshold not reached
            break;
        }
    }
}


// Evaluate work thresholds
bool evaluateThreshold(float totalWork, int stage) {
    if (stage == 1) return totalWork > FIRST_THRESHOLD;
    else if (stage == 2) return totalWork > SECOND_THRESHOLD;
    else if (stage == 3) return totalWork > THIRD_THRESHOLD;
    return false;
}

// Enter deep sleep and enable wake-up interrupt
void enterDeepSleep() {
    Serial.println("Entering deep sleep...");
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleWakeUpInterrupt, RISING);
    esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, 1);  // Enable wakeup on GPIO7
    esp_deep_sleep_start();
}

void scanI2CDevices() {
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }

    if (nDevices == 0) Serial.println("No I2C devices found\n");
    else Serial.println("done\n");
}
