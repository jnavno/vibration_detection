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
#define PHASE_DURATION 30000         // 30 seconds for each phase
#define NUM_PHASES 4                 // Total number of recording phases
#define PRE_TOGGLE_DELAY 1500        // Delay before toggling accel power

// MPU6050 Object
MPU6050 mpu;
volatile bool wakeup_flag = false;   // Flag to track wake-up by interrupt

// Function Prototypes
void initializeMPU();
void readAccelerometerDataForFFT();
void performFFT(float* inputBuffer, size_t length);

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

    // Initialize MPU6050
    initializeMPU();

    Serial.println("Starting vibration recording phases...");
}

void loop() {
    // Perform 4 recording phases, each 30 seconds long
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

// Function to initialize MPU6050
void initializeMPU() {
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);  // Stop execution if initialization fails
    }
    Serial.println("MPU6050 initialized.");
}

// Function to read accelerometer data and process it with FFT
void readAccelerometerDataForFFT() {
    uint16_t fifoCount = mpu.getFIFOCount();
    uint8_t fifoBuffer[64];

    if (fifoCount == 0) {
        Serial.println("No data in FIFO.");
        return;
    }

    // Read data from FIFO
    for (int i = 0; i < SAMPLES; i++) {
        if (fifoCount >= 6) {
            mpu.getFIFOBytes(fifoBuffer, 6);
            int16_t accelX = (fifoBuffer[0] << 8) | fifoBuffer[1];
            int16_t accelY = (fifoBuffer[2] << 8) | fifoBuffer[3];
            int16_t accelZ = (fifoBuffer[4] << 8) | fifoBuffer[5];

            // Use the X-axis for simplicity in FFT
            inputBuffer[i] = accelX / 16384.0;  // Convert raw value to g-force
        }
    }

    // Perform FFT on the input data
    performFFT(inputBuffer, SAMPLES);
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

    // Display the results of the FFT (only real components)
    for (size_t i = 0; i < length / 2 + 1; i++) {
        Serial.print("FFT bin ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(fftOutput[i].r);  // Only displaying real part for simplicity
    }

    free(cfg);  // Free the FFT configuration
}
