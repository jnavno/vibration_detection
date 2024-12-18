#include "sensorManager.h"
#include <Wire.h>
#include "MPU6050.h"
#include <variant.h>
#include <powerManager.h>
#include <spiffsManager.h>
#include "SPIFFS.h"
#include "FFT.h"
#include "FFT_signal.h"

MPU6050 mpu;
volatile bool wakeup_flag = false;
float inputBuffer[MAX_SAMPLES];
int remainingCycles = CYCLES_FOR_5_MIN;

// FFT thresholds and frequencies
float max_axe_magnitude = 0;
float max_saw_magnitude = 0;
float max_chainsaw_magnitude = 0;
float max_machete_magnitude = 0;

#define SAMPLING_FREQUENCY 5       // Adjustable
// Sampling Frequency = 1000 / (1 + Rate Divider)
// With mpu.setRate(99), the sampling frequency is 10 Hz (1000 / (1 + 99)).
// If a 5 Hz sampling rate is intended, set Rate Divider to 199 with mpu.setRate(199),
// which would yield 5 Hz (1000 / (1 + 199)), aligning with the defined SAMPLING_FREQUENCY.

#define AXE_MIN_FREQ 20.0
#define AXE_MAX_FREQ 60.0
#define SAW_MIN_FREQ 5.0
#define SAW_MAX_FREQ 30.0
#define CHAINSAW_MIN_FREQ 50.0
#define CHAINSAW_MAX_FREQ 250.0
#define MACHETE_MIN_FREQ 1.0       // Machete impact detection range
#define MACHETE_MAX_FREQ 3.0
#define MACHETE_THRESHOLD 5.0      // Threshold for machete impact detection
#define SOME_THRESHOLD 0.3

void performFFT();
void handleWakeUpInterrupt() {
    // Actions to perform upon wake-up
    wakeup_flag = true; // Set a flag or take any action needed upon waking
}


bool setupSensors() {
    powerCycleMPU(true);  // Power on the MPU via Vext
    delay(2000);          // Increased delay to ensure good power-up

    Wire.begin(41, 42);   // 38, 1 for prototype, (41,42) for released hat
    Wire.setClock(100000); // Required for I2C transfers stability

    if (!initializeMPU()) {
        Serial.println("MPU6050 initialization failed in setupSensors.");
        return false;
    }

    Serial.println("MPU6050 successfully initialized.");
    return true;
}

bool initializeMPU() {
    mpu.initialize();
    delay(100);  // Keep this for MPU stability
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed.");
        return false;
    }
    Serial.println("MPU6050 connected");

    // FIFO for accelerometer data configuration
    mpu.setFIFOEnabled(false);
    mpu.resetFIFO();
    mpu.setAccelFIFOEnabled(true);
    mpu.setFIFOEnabled(true);
    mpu.setRate(99);                                // ~10Hz sampling rate
    //mpu.setRate(199);                               // ~5Hz sampling rate
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // ±2g range
    delay(1000);                                    // Keep this to stabilize MPU
    return true;
}

void monitorSensors() {
    bool significantActivityDetected = false;

    if (remainingCycles <= 0) {
        quickBlinkAndHalt();  // Stop the system if out of cycles
        return;
    }

    unsigned long startTime = millis();
    while (millis() - startTime < 70000) {  // 70-second window for checking activity
        Serial.printf("Recording phase %d...\n", CYCLES_FOR_5_MIN - remainingCycles + 1);
        Serial.printf("%d remaining reading cycles\n", remainingCycles);

        toggleSensorPower(true);  // Power on MPU
        delay(1000);              // Stabilize

        if (!initializeMPU()) {
            Serial.println("MPU6050 initialization failed.");
            continue;
        }

        readAccelerometerDataForPhase(remainingCycles);
        
        // Run FFT analysis and check for activity
        performFFT();
        
        if (max_axe_magnitude > SOME_THRESHOLD || 
            max_chainsaw_magnitude > SOME_THRESHOLD || 
            max_machete_magnitude > MACHETE_THRESHOLD) {
            significantActivityDetected = true;
        }

        if (!significantActivityDetected && millis() - startTime >= 70000) {
            // If no activity is detected within 70 seconds, enter deep sleep
            enterDeepSleep();
        }

        toggleSensorPower(false);  // Power off MPU
        delay(5000);  // Delay for MPU stabilization
    }

    remainingCycles--;
}


void readAccelerometerDataForPhase(int phase) {
    uint8_t fifoBuffer[BLOCK_SIZE];
    int samplesRead = 0;
    unsigned long startMillis = millis();

    while (millis() - startMillis < PHASE_DURATION) {
        uint16_t fifoCount = mpu.getFIFOCount();

        if (fifoCount >= BLOCK_SIZE) {
            mpu.getFIFOBytes(fifoBuffer, BLOCK_SIZE);
            mpu.resetFIFO();

            for (int i = 0; i <= BLOCK_SIZE - 6 && samplesRead < MAX_SAMPLES; i += 6) {
                int16_t accelX = (fifoBuffer[i] << 8) | fifoBuffer[i + 1];
                inputBuffer[samplesRead++] = accelX / 16384.0; // Convert to g-force
            }
        } else if (fifoCount == 0) {
            delay(100);  // Wait for FIFO to fill
        }

        delay(1000 / SAMPLE_RATE);  // Control reading rate
    }
}

void performFFT() {
    fft_config_t *real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

    for (int i = 0; i < SAMPLES; i++) {
        real_fft_plan->input[i] = (i < MAX_SAMPLES) ? inputBuffer[i] : 0;
    }

    fft_execute(real_fft_plan);

    max_axe_magnitude = 0;
    max_saw_magnitude = 0;
    max_chainsaw_magnitude = 0;
    max_machete_magnitude = 0;

    Serial.println("FFT Results:");
    for (int i = 1; i < (SAMPLES / 2); i++) {
        float frequency = (i * SAMPLING_FREQUENCY) / SAMPLES;
        float magnitude = sqrt(pow(real_fft_plan->output[2 * i], 2) + pow(real_fft_plan->output[2 * i + 1], 2));

        Serial.printf("Frequency: %f Hz, Magnitude: %f\n", frequency, magnitude);

        if (frequency >= AXE_MIN_FREQ && frequency <= AXE_MAX_FREQ) {
            if (magnitude > max_axe_magnitude) max_axe_magnitude = magnitude;
        } else if (frequency >= SAW_MIN_FREQ && frequency <= SAW_MAX_FREQ) {
            if (magnitude > max_saw_magnitude) max_saw_magnitude = magnitude;
        } else if (frequency >= CHAINSAW_MIN_FREQ && frequency <= CHAINSAW_MAX_FREQ) {
            if (magnitude > max_chainsaw_magnitude) max_chainsaw_magnitude = magnitude;
        } else if (frequency >= MACHETE_MIN_FREQ && frequency <= MACHETE_MAX_FREQ) {
            if (magnitude > max_machete_magnitude) max_machete_magnitude = magnitude;
        }
    }

    fft_destroy(real_fft_plan);

    // Decision logic
    if (max_machete_magnitude > MACHETE_THRESHOLD) {
        Serial.println("Machete impact detected!");
    } else if (max_chainsaw_magnitude > SOME_THRESHOLD) {
        Serial.println("Chainsaw cutting detected!");
    } else if (max_axe_magnitude > SOME_THRESHOLD) {
        Serial.println("Hand axe/hatchet cutting detected!");
    } else if (max_saw_magnitude > SOME_THRESHOLD) {
        Serial.println("Handsaw cutting detected!");
    } else {
        Serial.println("No significant cutting activity detected.");
    }
}


bool checkFIFOOverflow() {
    if (mpu.getIntFIFOBufferOverflowStatus()) {
        Serial.println("FIFO overflow detected!");
        mpu.resetFIFO();
        return true;
    }
    return false;
}

void powerCycleMPU(bool on) {
    toggleSensorPower(on);
    delay(on ? PRE_TOGGLE_DELAY : 3000);
}

void enterDeepSleep() {
    Serial.println("Completed all cycles, entering deep sleep...");

    // Attach interrupt for wake-up on GPIO7 (assumed as INTERRUPT_PIN here)
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleWakeUpInterrupt, RISING);
    esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, 1);  // Enable wakeup on GPIO7 rising edge

    // Enter deep sleep
    esp_deep_sleep_start();
}