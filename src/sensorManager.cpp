#include "sensorManager.h"
#include <Wire.h>
#include "MPU6050.h"
#include <variant.h>
#include <powerManager.h>
#include <spiffsManager.h>
#include "SPIFFS.h"
#include "FFT.h"  // Include the new FFT library
#include "FFT_signal.h"  // Defines the input and output buffers

MPU6050 mpu;
volatile bool wakeup_flag = false;
float inputBuffer[MAX_SAMPLES];
int remainingCycles = CYCLES_FOR_5_MIN;

// Define FFT-related constants
#define SAMPLES 128
#define SAMPLING_FREQUENCY 5  // 5 Hz sampling frequency (adjust as needed)

// Define frequency ranges for chainsaw, axe, and handsaw detection
#define AXE_MIN_FREQ 20.0  // Frequency range for hand axe/hatchet
#define AXE_MAX_FREQ 60.0
#define SAW_MIN_FREQ 5.0   // Frequency range for handsaw
#define SAW_MAX_FREQ 30.0
#define CHAINSAW_MIN_FREQ 50.0  // Frequency range for chainsaw
#define CHAINSAW_MAX_FREQ 250.0
#define SOME_THRESHOLD 0.3  // Adjust threshold as per your application

float max_axe_magnitude = 0;
float max_saw_magnitude = 0;
float max_chainsaw_magnitude = 0;

void performFFT();  // Declare your custom function

void setupSensors() {
    Wire.begin(41, 42);        // Initialize I2C for MPU6050
    Wire.setClock(30000);      // Set I2C clock
    initializeMPU();           // Initialize MPU6050
}

bool initializeMPU() {
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        return false;
    }
    Serial.println("MPU6050 connected");

    // Reset and configure the FIFO
    mpu.setFIFOEnabled(false);
    mpu.resetFIFO();
    mpu.setAccelFIFOEnabled(true);
    mpu.setFIFOEnabled(true);
    mpu.setRate(99);                                // ~5Hz sampling rate
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // ±2g range
    delay(1000);                                    // Wait for MPU to stabilize
    return true;
}

void monitorSensors() {
    static int remainingCycles = CYCLES_FOR_5_MIN;
    if (remainingCycles <= 0) {
        quickBlinkAndHalt();  // Stop the system when limit is reached
        return;
    }

    for (int phase = 1; phase <= CYCLES_FOR_5_MIN; phase++) {
        Serial.printf("Recording phase %d...\n", phase);
        Serial.printf("%d remaining reading cycles\n", remainingCycles);

        bool phaseCompleted = false;
        int retryCount = 0;

        while (!phaseCompleted && retryCount < 3) {
            powerCycleMPU(true);
            if (!initializeMPU()) {
                Serial.println("MPU6050 initialization failed.");
                SPIFFSDebug("MPU initialization failed for phase ", phase);
                retryCount++;
                continue;
            }

            readAccelerometerDataForPhase(phase);

            bool loggingSuccess = logDataToSPIFFS(inputBuffer, MAX_SAMPLES, phase);
            if (loggingSuccess) {
                phaseCompleted = true;
                powerCycleMPU(false);

                // Perform FFT analysis to detect the type of cutting
                performFFT();
            } else {
                Serial.println("Failed to log data to SPIFFS. Retrying...");
                SPIFFSDebug("Failed to log data for phase ", phase);
                SPIFFS.end();
                if (!SPIFFS.begin(true)) {
                    Serial.println("SPIFFS remount failed. Skipping phase...");
                    break;
                }
            }
        }
        remainingCycles--;
        delay(5000);  // Wait before the next phase
    }
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
            delay(100);
        }

        delay(1000 / SAMPLE_RATE);  // Control reading rate
    }
}

void performFFT() {
    // Initialize FFT plan for real FFT
    fft_config_t *real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

    // Fill the input buffer with collected accelerometer data (inputBuffer)
    for (int i = 0; i < SAMPLES; i++) {
        real_fft_plan->input[i] = (i < MAX_SAMPLES) ? inputBuffer[i] : 0;  // Fill remaining with 0 if less samples
    }

    // Execute the FFT
    fft_execute(real_fft_plan);

    // Reset magnitudes for each detection type
    max_axe_magnitude = 0;
    max_saw_magnitude = 0;
    max_chainsaw_magnitude = 0;

    // Analyze FFT results and log to serial output
    Serial.println("FFT Results:");
    for (int i = 1; i < (SAMPLES / 2); i++) {  // Skip the first bin (DC component)
        float frequency = (i * SAMPLING_FREQUENCY) / SAMPLES;  // Calculate frequency for each bin
        float magnitude = sqrt(pow(real_fft_plan->output[2 * i], 2) + pow(real_fft_plan->output[2 * i + 1], 2));

        // Log frequencies and magnitudes of interest (optional)
        Serial.printf("Frequency: %f Hz, Magnitude: %f\n", frequency, magnitude);

        // Classify based on frequency range and store the max magnitude for each type
        if (frequency >= AXE_MIN_FREQ && frequency <= AXE_MAX_FREQ) {
            if (magnitude > max_axe_magnitude) {
                max_axe_magnitude = magnitude;
            }
        } else if (frequency >= SAW_MIN_FREQ && frequency <= SAW_MAX_FREQ) {
            if (magnitude > max_saw_magnitude) {
                max_saw_magnitude = magnitude;
            }
        } else if (frequency >= CHAINSAW_MIN_FREQ && frequency <= CHAINSAW_MAX_FREQ) {
            if (magnitude > max_chainsaw_magnitude) {
                max_chainsaw_magnitude = magnitude;
            }
        }
    }

    // Clean up FFT plan to free memory
    fft_destroy(real_fft_plan);

    // Decision logic based on detected magnitudes
    if (max_chainsaw_magnitude > SOME_THRESHOLD) {
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
    toggleAccelPower(on);  // Power on or off the MPU
    delay(on ? PRE_TOGGLE_DELAY : 3000);
}
