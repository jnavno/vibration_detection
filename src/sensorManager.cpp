#include "sensorManager.h"
#include <Wire.h>
#include "MPU6050.h"
#include <variant.h>
#include <powerManager.h>
#include <spiffsManager.h>
#include "SPIFFS.h"


MPU6050 mpu;
volatile bool wakeup_flag = false;
float inputBuffer[MAX_SAMPLES];
int remainingCycles = CYCLES_FOR_5_MIN;

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
