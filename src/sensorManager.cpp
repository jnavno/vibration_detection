#include <Wire.h>
#include "MPU6050.h"
#include "variant.h"
#include "FFT.h"
#include "FFT_signal.h"
#include "sensorManager.h"
#include "powerManager.h"
#include "DebugConfiguration.h"
#include "spiffsManager.h"
#include "SPIFFS.h"
#include <cstddef>
#include <stddef.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

// Use a constructor that passes the Wire pointer (if your library supports it)
// Ensure MPU6050_DEFAULT_ADDRESS is defined (commonly 0x68)
MPU6050 mpu(MPU6050_DEFAULT_ADDRESS, &Wire);

volatile bool wakeup_flag = false;
float inputBuffer[MAX_BUFFER_SIZE];

SFE_MAX1704X lipo(MAX1704X_MAX17048);
double voltage = 0; // LiPo voltage
double soc = 0;     // LiPo state-of-charge (SOC)
bool alert = false; // alert state

// FFT thresholds and frequencies
float max_saw_magnitude = 0;
float max_chainsaw_magnitude = 0;
float max_machete_magnitude = 0;
float overall_vibration_magnitude = 0;

int SAMPLE_RATE = DEFAULT_SAMPLE_RATE;  // Must be nonzero!
int SAMPLES = SAMPLE_RATE * SAMPLING_DURATION;

// Function prototypes
void monitorSensors();
void performFFT();
void normalizeGravity(float* accelX, float* accelY, float* accelZ);
void readAccelerometerDataForPhase(int phase);
void printResults();
bool initializeMAX();
void monitorBattery();
void setVoltageAlertThresholds(float minVoltage, float maxVoltage);

void handleWakeUpInterrupt() {
    if (!ENABLE_INTERRUPT) return;
    wakeup_flag = true;
    detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
}

// Interrupt Service Routine (ISR) for alert pin
void IRAM_ATTR alertISR() {
    alert = true;
}

void monitorBattery() {
    debug_println("Initializing Battery Monitoring...");
    bool maxStatus = initializeMAX();
    if (maxStatus) {
        voltage = lipo.getVoltage();
        soc = lipo.getSOC();
        debug_printf("Battery readings - voltage: %.2f, soc: %.2f, alert: %s\n",
                     voltage, soc, alert ? "true" : "false");
    } else {
        voltage = -1;
        soc = -1;
        alert = false;
        debug_println("Skipping battery readings due to MAX1704x initialization failure.");
    }
    delay(500);
    printResults();
}

void enterDeepSleep() {
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleWakeUpInterrupt, RISING);
    esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, 1);
    esp_deep_sleep_start();
}

void setVoltageAlertThresholds(float minVoltage, float maxVoltage) {
    uint8_t minThreshold = (uint8_t)(minVoltage / 0.020); // 20mV steps
    uint8_t maxThreshold = (uint8_t)(maxVoltage / 0.020);

    Wire.beginTransmission(0x36); // MAX17048 I2C Address
    Wire.write(0x14);             // VALRT register address
    Wire.write(minThreshold);
    Wire.write(maxThreshold);
    Wire.endTransmission();
}

bool setupSensors() {
    debug_println("Setting up sensors...");
    toggleSensorPower(true);
    delay(2000);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    // Attempt MPU initialization
    bool mpuStatus = initializeMPU();
    if (mpuStatus) {
        debug_println("MPU initialized successfully.");
    } else {
        debug_println("MPU initialization failed.");
    }

    // Attempt MAX1704x initialization
    bool maxStatus = initializeMAX();
    if (maxStatus) {
        debug_println("MAX1704x initialized successfully.");
    } else {
        debug_println("MAX1704x initialization failed.");
    }

    // If at least one sensor initialized, signal success with a single blink.
    if (mpuStatus || maxStatus) {
        debug_println("At least one sensor initialized successfully.");
        SensorInitOKBlink();  // This blink function (from powerManager) signals success.
        return true;
    } else {
        debug_println("No sensor initialized. Both MPU and MAX1704x failed.");
        return false;
    }
}


// Revised minimal MPU6050 initialization (like your old version)
bool initializeMPU() {
    const int maxRetries = 3;
    for (int attempt = 1; attempt <= maxRetries; attempt++){
        debug_printf("Initializing MPU6050, attempt %d...\n", attempt);
        mpu.initialize();
        delay(100);
        if (mpu.testConnection()) {
            debug_println("MPU6050 initialization successful!");
            mpu.setFIFOEnabled(false);     // Disable FIFO to configure it
            mpu.resetFIFO();               // Clear FIFO buffer
            mpu.setAccelFIFOEnabled(true); // Enable accelerometer FIFO
            mpu.setFIFOEnabled(true);      // Enable FIFO
            // Wake up and set clock source explicitly:
            mpu.setSleepEnabled(false);
            mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
            // Basic configuration (no FIFO for now)
            mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
            mpu.setRate(1000 / SAMPLE_RATE - 1);
            delay(1000);
            return true;
        } else {
            debug_println("MPU6050 initialization failed!");
            delay(1000);
        }
    }
    debug_println("MPU6050 failed to initialize after 3 attempts.");
    return false;
}

// MAX1704x initialization with retry (unchanged)
bool initializeMAX() {
    const int maxRetries = 3;
    pinMode(ALERT_LED_PIN, INPUT_PULLDOWN);
    pinMode(STATUS_LED_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ALERT_LED_PIN), alertISR, FALLING);

    lipo.quickStart();
    delay(100);
    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        debug_printf("MAX17048 initialization attempt: %d...\n", attempt);
        if (lipo.begin()) {
            debug_println("MAX17048 initialization successful!");
            lipo.setThreshold(20);
            setVoltageAlertThresholds(MIN_BATT_VOLTAGE, MAX_BATT_VOLTAGE);
            delay(1000);
            return true;
        } else {
            debug_printf("MAX17048 initialization failed! Attempt %d of %d\n", attempt, maxRetries);
            delay(1000);
        }
    }
    debug_println("MAX17048 init failed after 3 attempts.");
    return false;
}

void readAccelerometerDataForPhase(int phase) {
    uint8_t fifoBuffer[BLOCK_SIZE];
    int samplesRead = 0;
    unsigned long startMillis = millis();
    debug_printf("Starting accelerometer data read for phase %d\n", phase);

    while (millis() - startMillis < MONITOR_DURATION) {
        uint16_t fifoCount = mpu.getFIFOCount();
        debug_printf("FIFO Count: %d\n", fifoCount);
        if (fifoCount >= BLOCK_SIZE) {
            mpu.getFIFOBytes(fifoBuffer, BLOCK_SIZE);
            mpu.resetFIFO();
            debug_println("FIFO data read and FIFO reset.");
            for (int i = 0; i <= BLOCK_SIZE - 6 && samplesRead < SAMPLES; i += 6) {
                int16_t rawX = (fifoBuffer[i] << 8) | fifoBuffer[i + 1];
                int16_t rawY = (fifoBuffer[i + 2] << 8) | fifoBuffer[i + 3];
                int16_t rawZ = (fifoBuffer[i + 4] << 8) | fifoBuffer[i + 5];
                debug_printf("Raw Accelerometer X: %d\n", rawX);
                debug_printf("Raw Accelerometer Y: %d\n", rawY);
                debug_printf("Raw Accelerometer Z: %d\n", rawZ);
                float accelX = rawX / 16384.0;
                float accelY = rawY / 16384.0;
                float accelZ = rawZ / 16384.0;
                normalizeGravity(&accelX, &accelY, &accelZ);
                debug_printf("Normalized Gravity X: %f\n", accelX);
                debug_printf("Normalized Gravity Y: %f\n", accelY);
                debug_printf("Normalized Gravity Z: %f\n", accelZ);
                inputBuffer[samplesRead++] = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));
                debug_printf("Acceleration Magnitude: %f\n", inputBuffer[samplesRead - 1]);
            }
        } else if (fifoCount == 0) {
            debug_println("FIFO is empty, waiting for data...");
            delay(10);
        }
        debug_println("Waiting for next sample...");
        delay(1000 / SAMPLE_RATE);
    }
    logDataToSPIFFS(inputBuffer, samplesRead, phase);
    debug_printf("Finished reading accelerometer data for phase %d\n", phase);
}

bool significantActivityDetected() {
    return (max_chainsaw_magnitude > max_machete_magnitude) ||
           (max_machete_magnitude > max_saw_magnitude) ||
           (overall_vibration_magnitude > VIBRATION_THRESHOLD);
}

void monitorSensors() {
    unsigned long startTime = millis();
    debug_println("Starting 60-second monitoring...");
    monitorBattery();
    debug_println("Monitoring battery...");
    
    // Use a static variable to avoid repeatedly updating sensor registers.
    static int lastSampleRate = SAMPLE_RATE;

    while (millis() - startTime < MONITOR_DURATION) {
        debug_println("Collecting accelerometer data...");
        readAccelerometerDataForPhase(0);

        debug_println("Performing FFT analysis...");
        performFFT();

        debug_printf("max_machete_magnitude: %f\n", max_machete_magnitude);
        debug_printf("max_chainsaw_magnitude: %f\n", max_chainsaw_magnitude);
        debug_printf("max_saw_magnitude: %f\n", max_saw_magnitude);

        // Dynamically adjust sampling rate
        if (max_machete_magnitude > max_chainsaw_magnitude && max_machete_magnitude > max_saw_magnitude) {
            debug_println("Machete detected, adjusting sample rate to 200.");
            SAMPLE_RATE = 200;
        } else if (max_chainsaw_magnitude > max_machete_magnitude && max_chainsaw_magnitude > max_saw_magnitude) {
            debug_println("Chainsaw detected, adjusting sample rate to 400.");
            SAMPLE_RATE = 400;
        } else if (max_saw_magnitude > max_machete_magnitude) {
            debug_println("Handsaw detected, adjusting sample rate to 100.");
            SAMPLE_RATE = 100;
        } else {
            SAMPLE_RATE = DEFAULT_SAMPLE_RATE;
        }
        SAMPLES = SAMPLE_RATE * SAMPLING_DURATION;
        
        // Only update sensor's sample rate if it has changed:
        if (SAMPLE_RATE != lastSampleRate) {
            debug_printf("Updating MPU6050 sample rate from %d to %d\n", lastSampleRate, SAMPLE_RATE);
            mpu.setRate(1000 / SAMPLE_RATE - 1);
            lastSampleRate = SAMPLE_RATE;
        }

        debug_println("Logging data to SPIFFS...");
        logDataToSPIFFS(inputBuffer, SAMPLES, 0);
        debug_println("Data logged to SPIFFS successfully.");
        debug_println("Printing results...");
        printResults();
        delay(5000);
    }
    toggleSensorPower(false);
    enterDeepSleep();
}

void performFFT() {
    fft_config_t *real_fft_plan = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
    if (!real_fft_plan) {
        debug_println("FFT initialization failed!");
        return;
    }
    for (int i = 0; i < SAMPLES; i++) {
        real_fft_plan->input[i] = (i < MAX_BUFFER_SIZE) ? inputBuffer[i] : 0;
    }
    fft_execute(real_fft_plan);
    max_saw_magnitude = max_chainsaw_magnitude = max_machete_magnitude = 0;
    overall_vibration_magnitude = 0;
    for (int i = 1; i < (SAMPLES / 2); i++) {
        float frequency = (i * SAMPLE_RATE) / SAMPLES;
        float magnitude = sqrt(pow(real_fft_plan->output[2 * i], 2) + pow(real_fft_plan->output[2 * i + 1], 2));
        overall_vibration_magnitude += magnitude;
        if (frequency >= SAW_MIN_FREQ && frequency <= SAW_MAX_FREQ) {
            max_saw_magnitude = std::max(max_saw_magnitude, magnitude);
        } else if (frequency >= CHAINSAW_MIN_FREQ && frequency <= CHAINSAW_MAX_FREQ) {
            max_chainsaw_magnitude = std::max(max_chainsaw_magnitude, magnitude);
        } else if (frequency >= MACHETE_MIN_FREQ && frequency <= MACHETE_MAX_FREQ) {
            max_machete_magnitude = std::max(max_machete_magnitude, magnitude);
        }
    }
    fft_destroy(real_fft_plan);
}

void normalizeGravity(float* accelX, float* accelY, float* accelZ) {
    static bool gravityCalculated = false;
    static float gravityX = 0, gravityY = 0, gravityZ = 0;
    if (!gravityCalculated) {
        gravityX = *accelX;
        gravityY = *accelY;
        gravityZ = *accelZ;
        gravityCalculated = true;
    }
    *accelX -= gravityX;
    *accelY -= gravityY;
    *accelZ -= gravityZ;
}

void printResults() {
    debug_println("===== Sensor Results =====");
    debug_printf("Machete: %.2f\n", max_machete_magnitude);
    debug_printf("Chainsaw: %.2f\n", max_chainsaw_magnitude);
    debug_printf("Saw: %.2f\n", max_saw_magnitude);
    debug_printf("Voltage: %.2fV\n", voltage);
    debug_printf("Battery: %.2f%%\n", soc);
    debug_println("=========================");

    // Blink LED 5 times quickly (100ms ON, 100ms OFF) to signal results
    for (int i = 0; i < 5; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
    }
    lipo.clearAlert();
    alert = false;
}
