#include <Arduino.h>
#include "SPIFFS.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Pin Definitions
#define LED_PIN 3                    
#define ACCEL_PWR_PIN 5              
#define INTERRUPT_PIN GPIO_NUM_7     

// Timing Definitions
#define SAMPLES 128                  
#define PHASE_DURATION 30000  // 30 seconds for each phase
#define NUM_PHASES 3
#define BLOCK_SIZE 32                
#define SAMPLE_RATE 50
#define MAX_SAMPLES (SAMPLE_RATE * (PHASE_DURATION / 1000))
#define FILENAME_FORMAT "/data_phase_%d.csv"

// MPU6050 Object
MPU6050 mpu;
volatile bool wakeup_flag = false;  

float inputBuffer[MAX_SAMPLES];  // Buffer for accelerometer data

// Function Prototypes
bool initializeMPU();
void readAccelerometerDataForPhase(int phase);
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
    Wire.setClock(100000);  // Set I2C clock to 50kHz for stable communication
    delay(2000);

    setupSPIFFS();  // Initialize SPIFFS

    powerCycleMPU();
    if (!initializeMPU()) {
        Serial.println("MPU6050 initialization failed.");
        while (1);  // Halt if MPU initialization fails
    }

    Serial.println("Starting vibration recording phases...");
}

void loop() {
    for (int phase = 1; phase <= NUM_PHASES; phase++) {
        Serial.printf("Recording phase %d...\n", phase);

        // Re-initialize MPU before starting phase
        powerCycleMPU();
        if (!initializeMPU()) {
            Serial.println("MPU6050 initialization failed.");
            while (1);
        }

        // Collect accelerometer data for this phase
        readAccelerometerDataForPhase(phase);

        // Log data to SPIFFS and send over serial
        logDataToSPIFFS(inputBuffer, MAX_SAMPLES, phase);
        sendFileOverSerial(phase);

        delay(5000);  // Wait before the next phase
    }

    Serial.println("All phases complete. Waiting for next run...");
    delay(60000);  // 1 minute delay before repeating
}

void powerCycleMPU() {
    digitalWrite(ACCEL_PWR_PIN, LOW);
    delay(2000);  // Increase delay for MPU6050 stabilization
    digitalWrite(ACCEL_PWR_PIN, HIGH);
    delay(2000);  // Additional delay for power stabilization
}

bool initializeMPU() {
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        return false;
    }
    Serial.println("MPU6050 connected");

    // Reset and configure FIFO
    mpu.setFIFOEnabled(false);
    mpu.resetFIFO();
    mpu.setAccelFIFOEnabled(true);
    mpu.setFIFOEnabled(true);

    mpu.setRate(199);  // Set sample rate to ~5Hz
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // Set accelerometer range to ±2g
    delay(1000);  // Allow MPU6050 to settle
    return true;
}

void readAccelerometerDataForPhase(int phase) {
    uint8_t fifoBuffer[BLOCK_SIZE];  
    int samplesRead = 0;

    unsigned long startMillis = millis();  
    while (millis() - startMillis < PHASE_DURATION) {
        uint16_t fifoCount = mpu.getFIFOCount();

        if (fifoCount >= BLOCK_SIZE) {
            mpu.getFIFOBytes(fifoBuffer, BLOCK_SIZE);  

            for (int i = 0; i <= BLOCK_SIZE - 6 && samplesRead < MAX_SAMPLES; i += 6) {
                int16_t accelX = (fifoBuffer[i] << 8) | fifoBuffer[i + 1];
                inputBuffer[samplesRead++] = accelX / 16384.0;  // Convert to g-force
            }

            mpu.resetFIFO();  // Reset FIFO after reading
        }

        delay(1000 / SAMPLE_RATE);  // Control reading rate
    }
}

void setupSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return;
    }
    Serial.println("SPIFFS mounted successfully");
}

void logDataToSPIFFS(float* data, size_t length, int phase) {
    char filename[32];
    snprintf(filename, sizeof(filename), FILENAME_FORMAT, phase);

    File file = SPIFFS.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }

    file.println("Time(s), X-axis (g)");  // Optional CSV header

    for (size_t i = 0; i < length; i++) {
        file.printf("%f,%f\n", (i / SAMPLE_RATE), data[i]);  
    }

    file.close();
}

void sendFileOverSerial(int phase) {
    char filename[32];
    snprintf(filename, sizeof(filename), FILENAME_FORMAT, phase);

    File file = SPIFFS.open(filename, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }

    while (file.available()) {
        Serial.write(file.read());
    }

    file.close();
}