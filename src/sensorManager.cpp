#include "sensorManager.h"
#include <Wire.h>
#include "MPU6050.h"
#include <variant.h>
#include <powerManager.h>
#include <spiffsManager.h>
#include "SPIFFS.h"
#include "FFT.h"
#include "FFT_signal.h"

// Global variables and configurations
MPU6050 mpu;
volatile bool wakeup_flag = false;
float inputBuffer[MAX_SAMPLES]; // Input buffer for accelerometer data
int remainingCycles = MAX_CYCLES; // Initialize remaining cycles

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
#define MACHETE_MIN_FREQ 0.5       // Machete impact detection range
#define MACHETE_MAX_FREQ 3.0
#define MACHETE_THRESHOLD 100      // Threshold for machete impact detection
#define AXE_THRESHOLD 18
#define CHAINSAW_THRESHOLD 23
#define SAW_THRESHOLD 60
#define ENABLE_INTERRUPT false       // Control interrupt functionality

// Function prototypes
void performFFT();

void handleWakeUpInterrupt() {
    if (!ENABLE_INTERRUPT) return; // Skip the interrupt handling if interrupts are disabled
    wakeup_flag = true; // Set a flag or take any action needed upon waking
}

bool setupSensors() {
    toggleSensorPower(true);  // Power on the MPU via Vext
    delay(PRE_TOGGLE_DELAY);  // Increased delay to ensure good power-up

    Wire.begin(38, 1);   // 38, 1 for prototype, (41,42) for released hat
    Wire.setClock(100000);    // Required for I2C transfers stability

    if (!initializeMPU()) {
        Serial.println("MPU6050 initialization failed in setupSensors.");
        return false;
    }

    Serial.println("MPU6050 successfully initialized.");
    return true;
}

bool initializeMPU() {
    mpu.initialize();
    delay(100); // Keep this for MPU stability
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed.");
        return false;
    }

    Serial.println("MPU6050 connected");

    // Configure FIFO for accelerometer data
    mpu.setFIFOEnabled(false);
    mpu.resetFIFO();
    mpu.setAccelFIFOEnabled(true);
    mpu.setFIFOEnabled(true);
    mpu.setRate((1000 / SAMPLE_RATE) - 1);          // Set sampling rate
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // ±2g range
    delay(1000);                                    // Stabilization
    return true;
}

void monitorSensors() {
    int currentCycle = 1;          // Start with cycle 1
    bool significantActivityDetected = false;

    while (currentCycle <= MAX_CYCLES) {
        unsigned long currentTime = millis() / 1000; // Current timestamp in seconds
        const char* treeLocation = "Tree 42";        // Hardcoded tree location

        Serial.printf("Starting recording cycle %d of %d...\n", currentCycle, MAX_CYCLES);

        toggleSensorPower(true);  // Power on MPU
        delay(PRE_TOGGLE_DELAY);  // Stabilize sensor

        if (!initializeMPU()) {
            Serial.println("MPU6050 initialization failed. Entering deep sleep...");
            toggleSensorPower(false);
            enterDeepSleep(); // Fail-safe: If MPU cannot initialize, go to sleep
        }

        // Perform a single 10-second sampling phase
        readAccelerometerDataForPhase(currentCycle);
        performFFT(); // Perform FFT analysis after collecting data

        // Check for significant activity
        if (max_machete_magnitude > MACHETE_THRESHOLD ||
            max_chainsaw_magnitude > CHAINSAW_THRESHOLD ||
            max_axe_magnitude > AXE_THRESHOLD ||
            max_saw_magnitude > SAW_THRESHOLD) {
            significantActivityDetected = true;

            // Send alarm with cycle number, timestamp, and tree location
            Serial.printf("Significant activity detected! Sending alarm number %d at timestamp %lu seconds at %s.\n",
                          currentCycle, currentTime, treeLocation);
        } else {
            Serial.printf("No significant activity detected in cycle %d.\n", currentCycle);
        }

        toggleSensorPower(false); // Power off MPU
        delay(5000);              // Allow stabilization

        if (currentCycle == MAX_CYCLES) {
            break; // Exit after the final cycle
        }

        currentCycle++; // Increment cycle count after processing
    }

    // After processing all cycles
    if (significantActivityDetected) {
        Serial.println("Significant activity detected in at least one cycle. Entering deep sleep...");
    } else {
        Serial.println("No significant activity detected after all cycles. Entering deep sleep...");
    }

    toggleSensorPower(false); // Ensure MPU is powered off before sleeping
    enterDeepSleep();
}





void normalizeGravity(float* accelX, float* accelY, float* accelZ) {
    static bool gravityCalculated = false;
    static float gravityX = 0, gravityY = 0, gravityZ = 0;

    if (!gravityCalculated) {
        // Capture initial gravity vector
        gravityX = *accelX;
        gravityY = *accelY;
        gravityZ = *accelZ;
        gravityCalculated = true;
    }

    // Subtract gravity component
    *accelX -= gravityX;
    *accelY -= gravityY;
    *accelZ -= gravityZ;
}

void readAccelerometerDataForPhase(int phase) {
    uint8_t fifoBuffer[BLOCK_SIZE];
    int samplesRead = 0;
    unsigned long startMillis = millis();

    Serial.printf("Starting data collection for phase %d...\n", phase);

    while (millis() - startMillis < PHASE_DURATION) {
        uint16_t fifoCount = mpu.getFIFOCount();

        if (fifoCount >= BLOCK_SIZE) {
            mpu.getFIFOBytes(fifoBuffer, BLOCK_SIZE);
            mpu.resetFIFO();

            for (int i = 0; i <= BLOCK_SIZE - 6 && samplesRead < MAX_SAMPLES; i += 6) {
                int16_t rawX = (fifoBuffer[i] << 8) | fifoBuffer[i + 1];
                int16_t rawY = (fifoBuffer[i + 2] << 8) | fifoBuffer[i + 3];
                int16_t rawZ = (fifoBuffer[i + 4] << 8) | fifoBuffer[i + 5];

                float accelX = rawX / 16384.0; // Convert to g-force
                float accelY = rawY / 16384.0;
                float accelZ = rawZ / 16384.0;

                // Normalize for gravity
                normalizeGravity(&accelX, &accelY, &accelZ);

                // Compute combined magnitude
                inputBuffer[samplesRead++] = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));
            }
        } else if (fifoCount == 0) {
            delay(10); // Wait for FIFO to fill
        }

        delay(1000 / SAMPLE_RATE); // Control reading rate
    }

    Serial.printf("Phase %d: Collected %d samples.\n", phase, samplesRead);
    logDataToSPIFFS(inputBuffer, samplesRead, phase); // Save data to SPIFFS
}


void performFFT() {
    fft_config_t *real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

    // Copy input data to FFT buffer
    for (int i = 0; i < FFT_N; i++) {
        real_fft_plan->input[i] = (i < MAX_SAMPLES) ? inputBuffer[i] : 0;
    }

    fft_execute(real_fft_plan);

    max_axe_magnitude = 0;
    max_saw_magnitude = 0;
    max_chainsaw_magnitude = 0;
    max_machete_magnitude = 0;

    Serial.println("FFT Results:");
    for (int i = 1; i < (FFT_N / 2); i++) {
        float frequency = (i * SAMPLE_RATE) / FFT_N;
        float magnitude = sqrt(pow(real_fft_plan->output[2 * i], 2) + pow(real_fft_plan->output[2 * i + 1], 2));

        Serial.printf("Frequency: %.2f Hz, Magnitude: %.4f\n", frequency, magnitude);

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

    // Print maximum magnitudes for each tool
    Serial.printf("Max Axe Magnitude: %.4f\n", max_axe_magnitude);
    Serial.printf("Max Saw Magnitude: %.4f\n", max_saw_magnitude);
    Serial.printf("Max Chainsaw Magnitude: %.4f\n", max_chainsaw_magnitude);
    Serial.printf("Max Machete Magnitude: %.4f\n", max_machete_magnitude);

        // Decision logic
    if (max_machete_magnitude > MACHETE_THRESHOLD) {
        Serial.println("Machete impact detected!");
    } else if (max_chainsaw_magnitude > CHAINSAW_THRESHOLD) {
        Serial.println("Chainsaw cutting detected!");
    } else if (max_axe_magnitude > AXE_THRESHOLD) {
        Serial.println("Hand axe/hatchet cutting detected!");
    } else if (max_saw_magnitude > SAW_THRESHOLD) {
        Serial.println("Handsaw cutting detected!");
    } else {
        Serial.println("No significant cutting activity detected.");
    }
}

void enterDeepSleep() {
    Serial.println("Entering deep sleep...");
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleWakeUpInterrupt, RISING);
    esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, 1); // Enable wakeup on GPIO7 rising edge
    esp_deep_sleep_start();
}
