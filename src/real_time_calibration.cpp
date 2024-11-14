#include <Arduino.h>
#include "SPIFFS.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Pin Definitions
#define LED_PIN 3
#define ACCEL_PWR_PIN 5
#define INTERRUPT_PIN GPIO_NUM_7
#define STATUS_LED_PIN 4 // LED for quick blink after the limit is reached

// Timing Definitions
#define SAMPLES 128
#define PHASE_DURATION 30000 // 30 seconds for each phase
#define CYCLES_FOR_5_MIN 10  // 5 minutes = 10 cycles of 30 seconds
#define BLOCK_SIZE 32
#define SAMPLE_RATE 50
#define MAX_SAMPLES (SAMPLE_RATE * (PHASE_DURATION / 1000))
#define FILENAME_FORMAT "/data_phase_%d.csv"
#define PRE_TOGGLE_DELAY 1500 // Delay before toggling accelerometer power
#define MAX_SPIFFS_USAGE 30000 // Max limit of used space in bytes (30KB)

// MPU6050 Object
MPU6050 mpu;
volatile bool wakeup_flag = false;

float inputBuffer[MAX_SAMPLES]; // Buffer for accelerometer data
int remainingCycles = CYCLES_FOR_5_MIN;  // Track the remaining cycles

// Function Prototypes
bool initializeMPU();
void readAccelerometerDataForPhase(int phase);
void powerCycleMPU(bool on);
bool logDataToSPIFFS(float *data, size_t length, int phase);
void setupSPIFFS();
void sendFileOverSerial(int phase);
bool checkFIFOOverflow();
void blinkLED(int delayTime);
void SPIFFSDebug(const char *errorMessage, int phase);
void toggleAccelPower(bool state);
void checkSPIFFSSpace();
void quickBlinkAndHalt();

void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(ACCEL_PWR_PIN, OUTPUT);
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED_PIN, OUTPUT);

    Wire.begin(41, 42);
    Wire.setClock(30000); // I2C clock to 30kHz for stable communication
    delay(2000);

    setupSPIFFS(); // Initialize SPIFFS
    checkSPIFFSSpace();  // Check available space in SPIFFS

    // Power on the accelerometer
    toggleAccelPower(true);
    delay(PRE_TOGGLE_DELAY); // Allow time for accelerometer to power up

    powerCycleMPU(true);

    if (!initializeMPU())
    {
        Serial.println("MPU6050 initialization failed.");
        while (1); // Halt if MPU initialization fails
    }
    Serial.println("Starting vibration recording phases...");
}

void loop()
{
    // Stop logging if the 5-minute limit (10 cycles) is reached
    if (remainingCycles <= 0)
    {
        quickBlinkAndHalt();  // Blink LED and stop the system gracefully
        return;
    }

    for (int phase = 1; phase <= CYCLES_FOR_5_MIN; phase++)
    {
        Serial.printf("Recording phase %d...\n", phase);
        Serial.printf("%d remaining reading cycles\n", remainingCycles);

        bool phaseCompleted = false; // Track if the current phase completed successfully
        int retryCount = 0;          // Retry counter for the current phase

        while (!phaseCompleted && retryCount < 3)
        {
            // Re-initialize MPU before starting phase
            powerCycleMPU(true);
            if (!initializeMPU())
            {
                Serial.println("MPU6050 initialization failed.");
                SPIFFSDebug("MPU initialization failed for phase ", phase); // Log to SPIFFS debug
                retryCount++;
                continue; // Retry the phase
            }
            Serial.println("MPU6050 reinitialized for data readings.");

            // Collect accelerometer data for this phase
            readAccelerometerDataForPhase(phase);

            // Attempting to log data to SPIFFS
            bool loggingSuccess = false;
            for (int loggingRetry = 0; loggingRetry < 3; loggingRetry++)
            { // Retry up to 3 times for SPIFFS logging
                loggingSuccess = logDataToSPIFFS(inputBuffer, MAX_SAMPLES, phase);
                if (loggingSuccess)
                {
                    // logging was successful
                    break; // Exit retry loop if logging was successful
                }
                else
                {
                    Serial.println("Failed to log data to SPIFFS. Retrying...");
                    SPIFFSDebug("Failed to log data for phase ", phase); // Log SPIFFS failure for phase
                }
            }

            if (!loggingSuccess)
            {
                retryCount++;
                SPIFFSDebug("Failed logging for phase after retries: ", phase); // Log SPIFFS error
                // Perform SPIFFS debug: remount, check space, or reset SPIFFS if necessary
                SPIFFS.end();
                if (!SPIFFS.begin(true))
                {
                    Serial.println("SPIFFS remount failed. Skipping phase...");
                    retryCount = 3; // Break retry loop if SPIFFS remount failed
                }
                continue; // Retry the current phase if logging failed
            }
            sendFileOverSerial(phase);

            phaseCompleted = true;

            powerCycleMPU(false);  // Turn off MPU after phase completes

            // Ensure we reset the MPU again to avoid errors in subsequent phases
            Serial.println("Resetting MPU6050 for the next phase...");
        }
        if (!phaseCompleted)
        {
            Serial.printf("Phase %d could not complete after 3 retries. Moving to the next phase...\n", phase);
            SPIFFSDebug("Phase not completed after retries: ", phase); // Log failure to debug
        }
        remainingCycles--;
        delay(5000); // Wait before the next phase
    }

    Serial.println("All phases complete. Waiting for next run...");
    delay(5000); // 10 seconds delay before repeating
}

void checkSPIFFSSpace() {
    Serial.printf("Total SPIFFS space: %u bytes\n", SPIFFS.totalBytes());
    Serial.printf("Used SPIFFS space: %u bytes\n", SPIFFS.usedBytes());
}

// Function to log debug information to SPIFFS
void SPIFFSDebug(const char *errorMessage, int phase)
{
    File debugFile = SPIFFS.open("/debug_log.txt", FILE_APPEND);
    if (!debugFile)
    {
        Serial.println("Failed to open debug log file.");
        return;
    }
    debugFile.printf("Error: %s during phase %d\n", errorMessage, phase);
    debugFile.close();
}

void toggleAccelPower(bool state)
{
    digitalWrite(ACCEL_PWR_PIN, state ? LOW : HIGH);
}

// Power cycle MPU6050 based on the 'on' state
void powerCycleMPU(bool on)
{
    if (on)
    {
        toggleAccelPower(true);  // Turn MPU on
        delay(PRE_TOGGLE_DELAY); // Ensure MPU has time to power up
    }
    else
    {
        toggleAccelPower(false); // Turn MPU off
        delay(3000);             // Allow enough time for the MPU to power down
    }
}

bool initializeMPU()
{
    mpu.initialize();
    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed");
        return false;
    }
    Serial.println("MPU6050 connected");

    // Reset and configure FIFO
    mpu.setFIFOEnabled(false);
    mpu.resetFIFO();
    mpu.setAccelFIFOEnabled(true);
    mpu.setFIFOEnabled(true);

    mpu.setRate(99);                                // Set sample rate to ~10Hz
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // Set accelerometer range to ±2g
    delay(1000);                                    // Allow MPU6050 to settle
    return true;
}

bool checkFIFOOverflow()
{
    if (mpu.getIntFIFOBufferOverflowStatus())
    {
        Serial.println("FIFO overflow detected!");
        mpu.resetFIFO(); // Reset FIFO to avoid I2C errors
        return true;
    }
    return false;
}

void readAccelerometerDataForPhase(int phase)
{
    uint8_t fifoBuffer[BLOCK_SIZE];
    int samplesRead = 0;

    unsigned long startMillis = millis();
    while (millis() - startMillis < PHASE_DURATION)
    {
        uint16_t fifoCount = mpu.getFIFOCount();

        if (fifoCount >= BLOCK_SIZE)
        {
            // Read the available data
            mpu.getFIFOBytes(fifoBuffer, BLOCK_SIZE);

            // Reset FIFO to prevent overflow after reading
            mpu.resetFIFO();

            // Process the data from FIFO
            for (int i = 0; i <= BLOCK_SIZE - 6 && samplesRead < MAX_SAMPLES; i += 6)
            {
                int16_t accelX = (fifoBuffer[i] << 8) | fifoBuffer[i + 1];
                inputBuffer[samplesRead++] = accelX / 16384.0; // Convert to g-force
            }

        } else if (fifoCount == 0) {
            // If no data is in the FIFO, wait a bit and try again
            delay(100);
        }

        delay(1000 / SAMPLE_RATE); // Control reading rate
    }
}

void setupSPIFFS()
{
    if (!SPIFFS.begin(true))
    {
        Serial.println("Failed to mount SPIFFS");
        return;
    }
    Serial.println("SPIFFS mounted successfully");
}

bool logDataToSPIFFS(float *data, size_t length, int phase)
{
    char filename[32];
    snprintf(filename, sizeof(filename), FILENAME_FORMAT, phase);

    Serial.printf("Attempting to open file: %s\n", filename);

    File file = SPIFFS.open(filename, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return false;
    }

    Serial.println("File opened successfully, writing data...");

    file.println("Time(s), X-axis (g)"); // Optional CSV header

    for (size_t i = 0; i < length; i++)
    {
        file.printf("%f,%f\n", ((float)i / SAMPLE_RATE), data[i]);
    }

    file.close();

    Serial.println("Data successfully written to SPIFFS.");
    return true;
}

// Blinks the LED quickly for 30 seconds and halts further operations
void quickBlinkAndHalt()
{
    Serial.println("5-minute data logging limit reached. Press reset to restart.");

    // Blink the status LED quickly for 30 seconds
    unsigned long blinkStart = millis();
    while (millis() - blinkStart < 30000)  // 30 seconds of blinking
    {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(200);  // Blink on
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(200);  // Blink off
    }

    // Halt further operations
    Serial.println("Data logging halted. Press reset to restart.");
    while (true);  // Infinite loop waiting for manual reset
}

void sendFileOverSerial(int phase)
{
    char filename[32];
    snprintf(filename, sizeof(filename), FILENAME_FORMAT, phase);

    File file = SPIFFS.open(filename, FILE_READ);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    while (file.available())
    {
        Serial.write(file.read());
    }

    file.close();
}

void blinkLED(int delayTime)
{
    digitalWrite(LED_PIN, HIGH); // Turn LED on
    delay(delayTime);
    digitalWrite(LED_PIN, LOW); // Turn LED off
    delay(delayTime);
}