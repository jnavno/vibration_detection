#include <Arduino.h>
#include "SPIFFS.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "kiss_fftr.h"  // Including the local KissFFT header

// Pin Definitions
#define LED_PIN 3                    // Define the LED pin (GPIO3)
#define ACCEL_PWR_PIN 5              // Define the pin to control the MOSFET
#define INTERRUPT_PIN GPIO_NUM_7     // GPIO7 for interrupt from the shake sensor

// Timing Definitions
#define SAMPLES 128                  // Number of samples for FFT analysis
#define PHASE_DURATION 10000         // 10 seconds for each phase (adjusted to avoid overflow)
#define NUM_PHASES 4                 // Total number of recording phases
#define PRE_TOGGLE_DELAY 1500        // Delay before toggling accel power
#define INIT_DELAY 1000              // Delay after powering on the accelerometer
#define BLOCK_SIZE 32                // Reading data from FIFO in blocks of 32 bytes

// MPU6050 Object
MPU6050 mpu;
volatile bool wakeup_flag = false;   // Flag to track wake-up by interrupt

// Function Prototypes
bool initializeMPU();
void readAccelerometerDataForFFT();
void performFFT(float* inputBuffer, size_t length);
bool checkFIFOOverflow();
void powerCycleMPU();
void logDataToSPIFFS(float* data, size_t length);
void setupSPIFFS();

// FFT-related variables
float inputBuffer[SAMPLES];    // Input buffer for accelerometer data
kiss_fft_cpx fftOutput[SAMPLES / 2 + 1];  // Output of FFT

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);  // Set LED pin as output
    pinMode(ACCEL_PWR_PIN, OUTPUT);
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);  // Set interrupt pin as input

    Wire.begin(41, 42);
    Wire.setClock(100000);  // Set I2C to 100kHz for stability
    delay(2000);

    // Initialize SPIFFS
    setupSPIFFS();

    // Power cycle the MPU6050 to reset any stale state
    powerCycleMPU();

    // Initialize MPU6050
    initializeMPU();

    Serial.println("Starting vibration recording phases...");
}

void loop() {
    // Perform 4 recording phases, each 10 seconds long
    for (int phase = 1; phase <= NUM_PHASES; phase++) {
        Serial.print("Recording phase ");
        Serial.println(phase);

        // Collect and process accelerometer data
        readAccelerometerDataForFFT();

        // Wait for the next phase
        delay(PHASE_DURATION);
    }

    // Indicate that calibration is complete
    Serial.println("Vibration recording complete. Ready for analysis.");
    delay(60000);  // Wait a minute before repeating
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

    // Set sample rate to 5Hz (for example)
    mpu.setRate(199);

    // Optionally set accelerometer range (default is ±2g)
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    delay(1000);  // Allow MPU to settle
    return true;
}

void toggleAccelPower(bool state) {
    digitalWrite(ACCEL_PWR_PIN, state ? LOW : HIGH);
}

void powerCycleMPU() {
    toggleAccelPower(false);  // Turn off MPU6050 (P-CHANNEL mosfet)
    delay(1000);              // Wait for the MPU to power down
    toggleAccelPower(true);   // Turn MPU6050 back on (P-CHANNEL mosfet)
    delay(PRE_TOGGLE_DELAY);  // Allow time for the MPU to power up
}

void setupSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return;
    }
    Serial.println("SPIFFS mounted successfully");
}

void logDataToSPIFFS(float* data, size_t length) {
    File file = SPIFFS.open("/data.txt", FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }

    for (size_t i = 0; i < length; i++) {
        file.println(data[i]);
    }

    file.close();
}

void readAccelerometerDataForFFT() {
    uint8_t fifoBuffer[BLOCK_SIZE];  // Block size for reading FIFO in small chunks
    int16_t accelX, accelY, accelZ;
    int samplesRead = 0;  // Counter for how many samples have been read
    float inputBuffer[SAMPLES];  // Input buffer for FFT processing

    while (samplesRead < SAMPLES) {
        uint16_t fifoCount = mpu.getFIFOCount();  // Get the current FIFO count
        Serial.print("FIFO count: ");
        Serial.println(fifoCount);

        if (fifoCount == 0) {
            Serial.println("No data in FIFO.");
            delay(100);  // Small delay to allow more data to accumulate
            continue;
        }

        // Check for FIFO overflow
        if (checkFIFOOverflow()) {
            Serial.println("FIFO overflow detected. Resetting FIFO.");
            mpu.resetFIFO();  // Reset FIFO if overflow is detected
            samplesRead = 0;  // Restart sample reading due to overflow
            break;
        }

        // Read in BLOCK_SIZE chunks while we have enough data in the FIFO
        while (fifoCount >= BLOCK_SIZE && samplesRead < SAMPLES) {
            mpu.getFIFOBytes(fifoBuffer, BLOCK_SIZE);  // Read FIFO data
            fifoCount -= BLOCK_SIZE;  // Decrease the count by the block size

            // Process accelerometer data (6 bytes per reading: X, Y, Z)
            for (int i = 0; i <= BLOCK_SIZE - 6 && samplesRead < SAMPLES; i += 6) {
                accelX = (fifoBuffer[i] << 8) | fifoBuffer[i + 1];
                accelY = (fifoBuffer[i + 2] << 8) | fifoBuffer[i + 3];
                accelZ = (fifoBuffer[i + 4] << 8) | fifoBuffer[i + 5];

                // Store X-axis data in the input buffer for FFT processing
                inputBuffer[samplesRead] = accelX / 16384.0;  // Convert raw value to g-force
                samplesRead++;
            }
        }

        // Reset the FIFO after every read to clear out old data
        mpu.resetFIFO();
        Serial.println("FIFO reset after data read.");

        // Delay between readings to mimic the original 11-second inspection period
        delay(1000);  // Adjust this delay to manage the flow of data correctly
    }

    // Perform FFT once enough samples are collected
    if (samplesRead == SAMPLES) {
        Serial.println("Performing FFT...");
        performFFT(inputBuffer, SAMPLES);
    }
}





// Function to check for FIFO overflow
bool checkFIFOOverflow() {
    if (mpu.getIntFIFOBufferOverflowStatus()) {
        Serial.println("FIFO overflow detected!");
        return true;
    }
    return false;
}

// Function to perform FFT using kissfft
void performFFT(float* inputBuffer, size_t length) {
    kiss_fftr_cfg cfg = kiss_fftr_alloc(length, 0, NULL, NULL);
    if (cfg == NULL) {
        Serial.println("Failed to allocate FFT configuration.");
        return;
    }

    // Perform the FFT
    kiss_fftr(cfg, inputBuffer, fftOutput);

    // Display the results of the FFT (real components)
    Serial.println("Frequency components (Hz):");
    for (size_t i = 0; i < length / 2 + 1; i++) {
        // Convert bin to frequency and output the real magnitude
        float frequency = (float)i * 1000.0 / SAMPLES;
        Serial.print(frequency);
        Serial.print(" Hz: ");
        Serial.println(fftOutput[i].r);  // Real part
    }

    free(cfg);  // Free the FFT configuration
}
