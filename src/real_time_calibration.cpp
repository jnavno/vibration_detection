#include <Arduino.h>
#include "SPIFFS.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "kiss_fftr.h"

// Pin Definitions
#define LED_PIN 3                    
#define ACCEL_PWR_PIN 5              
#define INTERRUPT_PIN GPIO_NUM_7     

// Timing Definitions
#define SAMPLES 128                  
#define PHASE_DURATION 30000         // 30 seconds for each phase
#define NUM_PHASES 3                 // Total number of recording phases
#define BLOCK_SIZE 32                
#define SAMPLE_RATE 50               // 50 Hz sample rate
#define MAX_SAMPLES (SAMPLE_RATE * (PHASE_DURATION / 1000))  // 1500 samples for 30s at 50Hz
#define FILENAME_FORMAT "/data_phase_%d.csv"

// MPU6050 Object
MPU6050 mpu;
volatile bool wakeup_flag = false;  

// FFT-related variables
float inputBuffer[MAX_SAMPLES];    
kiss_fft_cpx fftOutput[SAMPLES / 2 + 1];  

// Function Prototypes
bool initializeMPU();
void readAccelerometerDataForPhase(int phase);
void performFFT(float* inputBuffer, size_t length);
bool checkFIFOOverflow();
void powerCycleMPU();
void logDataToSPIFFS(float* data, size_t length, int phase);
void setupSPIFFS();
void sendFileOverSerial(int phase);

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);  
    pinMode(ACCEL_PWR_PIN, OUTPUT);
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);  

    Wire.begin(41, 42);
    Wire.setClock(100000);  
    delay(2000);

    // Initialize SPIFFS
    setupSPIFFS();

    // Power cycle the MPU6050 to reset any stale state
    powerCycleMPU();

    // Initialize MPU6050
    if (!initializeMPU()) {
        Serial.println("MPU6050 initialization failed.");
        while (1);  // Halt if MPU initialization fails
    }

    Serial.println("Starting vibration recording phases...");
}

void loop() {
    // Run through the 3 phases, each of 30 seconds
    for (int phase = 1; phase <= NUM_PHASES; phase++) {
        Serial.printf("Recording phase %d...\n", phase);

        // Power cycle MPU before starting phase
        powerCycleMPU();

        // Collect and process accelerometer data
        readAccelerometerDataForPhase(phase);

        // Log data to SPIFFS
        logDataToSPIFFS(inputBuffer, MAX_SAMPLES, phase);

        // Send the file over serial after logging
        sendFileOverSerial(phase);

        // Wait before the next phase
        Serial.printf("Phase %d complete. Waiting before next phase...\n", phase);
        delay(5000);  // Delay between phases
    }

    Serial.println("All phases complete. Waiting for next run...");
    delay(60000);  // Delay before starting the next recording cycle
}

bool initializeMPU() {
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        return false;
    }

    // Reset and configure FIFO
    mpu.setFIFOEnabled(false);     
    mpu.resetFIFO();               
    mpu.setAccelFIFOEnabled(true); 
    mpu.setFIFOEnabled(true);      

    // Set sample rate to ~50Hz
    mpu.setRate(19);  // (1kHz / (1 + rate)) => 50Hz

    // Set accelerometer range (±2g by default)
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    delay(1000);  // Allow MPU to settle
    return true;
}

void powerCycleMPU() {
    digitalWrite(ACCEL_PWR_PIN, LOW);  
    delay(1000);                      
    digitalWrite(ACCEL_PWR_PIN, HIGH); 
    delay(1500);                      
}

void setupSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return;
    }
    Serial.println("SPIFFS mounted successfully");
}

void readAccelerometerDataForPhase(int phase) {
    uint8_t fifoBuffer[BLOCK_SIZE];  
    int16_t accelX, accelY, accelZ;
    int samplesRead = 0;  

    unsigned long startMillis = millis();  
    while (millis() - startMillis < PHASE_DURATION) {
        uint16_t fifoCount = mpu.getFIFOCount();  

        if (fifoCount >= BLOCK_SIZE) {
            mpu.getFIFOBytes(fifoBuffer, BLOCK_SIZE);  

            for (int i = 0; i <= BLOCK_SIZE - 6 && samplesRead < MAX_SAMPLES; i += 6) {
                accelX = (fifoBuffer[i] << 8) | fifoBuffer[i + 1];
                accelY = (fifoBuffer[i + 2] << 8) | fifoBuffer[i + 3];
                accelZ = (fifoBuffer[i + 4] << 8) | fifoBuffer[i + 5];

                inputBuffer[samplesRead++] = accelX / 16384.0;  // Convert to g-force
                if (samplesRead >= MAX_SAMPLES) break;  
            }

            mpu.resetFIFO();  
        }

        delay(1000 / SAMPLE_RATE);  
    }

    Serial.printf("Phase %d: Collected %d samples\n", phase, samplesRead);
}

void logDataToSPIFFS(float* data, size_t length, int phase) {
    char filename[32];
    snprintf(filename, sizeof(filename), FILENAME_FORMAT, phase);

    File file = SPIFFS.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }

    file.println("Time(s), X-axis (g)");  

    for (size_t i = 0; i < length; i++) {
        file.printf("%f,%f\n", (i / SAMPLE_RATE), data[i]);  
    }

    file.close();
    Serial.printf("Data saved to %s\n", filename);
}

void sendFileOverSerial(int phase) {
    char filename[32];
    snprintf(filename, sizeof(filename), FILENAME_FORMAT, phase);

    File file = SPIFFS.open(filename, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.printf("Sending file: %s\n", filename);
    while (file.available()) {
        Serial.write(file.read());
    }

    file.close();
    Serial.println("\nFile transfer complete.");
}

// Function to check for FIFO overflow
bool checkFIFOOverflow() {
    if (mpu.getIntFIFOBufferOverflowStatus()) {
        Serial.println("FIFO overflow detected!");
        return true;
    }
    return false;
}

// Function to perform FFT using kissfft (optional, if needed)
void performFFT(float* inputBuffer, size_t length) {
    kiss_fftr_cfg cfg = kiss_fftr_alloc(length, 0, NULL, NULL);
    if (cfg == NULL) {
        Serial.println("Failed to allocate FFT configuration.");
        return;
    }

    kiss_fftr(cfg, inputBuffer, fftOutput);

    Serial.println("Frequency components (Hz):");
    for (size_t i = 0; i < length / 2 + 1; i++) {
        float frequency = (float)i * 1000.0 / SAMPLES;
        Serial.printf("%f Hz: %f\n", frequency, fftOutput[i].r);  
    }

    free(cfg);
}