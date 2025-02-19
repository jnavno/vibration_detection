#include <Wire.h>
#include "MPU6050.h"
#include "variant.h"
#include "FFT.h"
#include "FFT_signal.h"
#include "sensorManager.h"
#include "powerManager.h"
#include "spiffsManager.h"
#include "SPIFFS.h"
#include <cstddef>
#include <stddef.h>

void monitorFor60Sec();
void performFFT();
void normalizeGravity(float* accelX, float* accelY, float* accelZ);
void readAccelerometerDataForPhase(int phase);

MPU6050 mpu;
volatile bool wakeup_flag = false;
float inputBuffer[MAX_BUFFER_SIZE];

// FFT thresholds and frequencies
float max_saw_magnitude = 0;
float max_chainsaw_magnitude = 0;
float max_machete_magnitude = 0;
float overall_vibration_magnitude = 0;

int SAMPLE_RATE = DEFAULT_SAMPLE_RATE;  // Start in low-power mode
int SAMPLES = SAMPLE_RATE * SAMPLING_DURATION;

void handleWakeUpInterrupt() {
    if (!ENABLE_INTERRUPT) return;
    wakeup_flag = true;
    detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
}

void enterDeepSleep() {
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleWakeUpInterrupt, RISING);
    esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, 1);
    esp_deep_sleep_start();
}

bool setupSensors() {
    toggleSensorPower(true);
    delay(2000);
    Wire.begin(41, 42);
    Wire.setClock(100000);
    if (!initializeMPU()) {
        return false;
    }
    return true;
}

bool initializeMPU() {
    mpu.initialize();
    delay(100);
    if (!mpu.testConnection()) {
        return false;
    }
    mpu.setFIFOEnabled(false);
    mpu.resetFIFO();
    mpu.setAccelFIFOEnabled(true);
    mpu.setFIFOEnabled(true);
    mpu.setRate(1000 / SAMPLE_RATE - 1);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    delay(1000);
    return true;
}

void readAccelerometerDataForPhase(int phase) {
    uint8_t fifoBuffer[BLOCK_SIZE];
    int samplesRead = 0;
    unsigned long startMillis = millis();

    while (millis() - startMillis < MONITOR_DURATION) {
        uint16_t fifoCount = mpu.getFIFOCount();
        if (fifoCount >= BLOCK_SIZE) {
            mpu.getFIFOBytes(fifoBuffer, BLOCK_SIZE);
            mpu.resetFIFO();
            for (int i = 0; i <= BLOCK_SIZE - 6 && samplesRead < SAMPLES; i += 6) {
                int16_t rawX = (fifoBuffer[i] << 8) | fifoBuffer[i + 1];
                int16_t rawY = (fifoBuffer[i + 2] << 8) | fifoBuffer[i + 3];
                int16_t rawZ = (fifoBuffer[i + 4] << 8) | fifoBuffer[i + 5];
                float accelX = rawX / 16384.0;
                float accelY = rawY / 16384.0;
                float accelZ = rawZ / 16384.0;
                normalizeGravity(&accelX, &accelY, &accelZ);
                inputBuffer[samplesRead++] = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));
            }
        } else if (fifoCount == 0) {
            delay(10);
        }
        delay(1000 / SAMPLE_RATE);
    }
    logDataToSPIFFS(inputBuffer, samplesRead, phase);
}


bool significantActivityDetected() {
    return (max_chainsaw_magnitude > max_machete_magnitude) ||
           (max_machete_magnitude > max_saw_magnitude) ||
           (overall_vibration_magnitude > VIBRATION_THRESHOLD);
}

void monitorFor60Sec() {
    unsigned long startTime = millis();
    while (millis() - startTime < MONITOR_DURATION) {
        toggleSensorPower(true);
        delay(1000);
        if (!initializeMPU()) {
            continue;
        }
        readAccelerometerDataForPhase(0);
        performFFT();

        // Adjust sampling rate dynamically
        if (max_machete_magnitude > max_chainsaw_magnitude && max_machete_magnitude > max_saw_magnitude) {
            SAMPLE_RATE = 200; // Machete detected
        } else if (max_chainsaw_magnitude > max_machete_magnitude && max_chainsaw_magnitude > max_saw_magnitude) {
            SAMPLE_RATE = 400; // Chainsaw detected
        } else if (max_saw_magnitude > max_machete_magnitude) {
            SAMPLE_RATE = 100; // Handsaw detected
        } else {
            SAMPLE_RATE = DEFAULT_SAMPLE_RATE;  // Default low-power mode
        }

        SAMPLES = SAMPLE_RATE * SAMPLING_DURATION; // Update dynamic sample size
        mpu.setRate(1000 / SAMPLE_RATE - 1); // Apply new sample rate
        logDataToSPIFFS(inputBuffer, SAMPLES, 0);
        toggleSensorPower(false);
        delay(5000);
    }
    enterDeepSleep();
}

void performFFT() {
    fft_config_t *real_fft_plan = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
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
