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
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

void monitorFor60Sec();
void performFFT();
void normalizeGravity(float* accelX, float* accelY, float* accelZ);
void readAccelerometerDataForPhase(int phase);
void printResults();
bool initializeMAX();
void monitorBattery();
void setVoltageAlertThresholds(float minVoltage, float maxVoltage);

MPU6050 mpu;
volatile bool wakeup_flag = false;
float inputBuffer[MAX_BUFFER_SIZE];

SFE_MAX1704X lipo(MAX1704X_MAX17048);
double voltage = 0; // LiPo voltage
double soc = 0; // LiPo state-of-charge (SOC)
bool alert = false; // alert state

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

// Interrupt Service Routine (ISR) for alert pin
void IRAM_ATTR alertISR() {
    alert = true;
}

void monitorBattery() {

    toggleSensorPower(true);
        delay(1000);
        if (!initializeMAX()) {
            voltage = lipo.getVoltage(); // Get battery voltage
            soc = lipo.getSOC();         // Get battery SOC
            bool alertStatus = lipo.getAlert(); // Get alert state
        }
    toggleSensorPower(false);
    delay(500);
    printResults();
}

void enterDeepSleep() {
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleWakeUpInterrupt, RISING);
    esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, 1);
    esp_deep_sleep_start();
}

void setVoltageAlertThresholds(float minVoltage, float maxVoltage) {
    uint8_t minThreshold = (uint8_t)(minVoltage / 0.020); // Convert to 20mV steps
    uint8_t maxThreshold = (uint8_t)(maxVoltage / 0.020);

    Wire.beginTransmission(0x36); // MAX17048 I2C Address
    Wire.write(0x14); // VALRT register address
    Wire.write(minThreshold); // Set minimum voltage threshold
    Wire.write(maxThreshold); // Set maximum voltage threshold
    Wire.endTransmission();
}

bool setupSensors() {
    toggleSensorPower(true);
    delay(2000);
    Wire.begin(41, 42);
    Wire.setClock(100000);
    return initializeMPU() && initializeMAX();
}

// Accelerometer
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
    quickStatusOnBlink();
    return true;
}

// Battery Gauge
bool initializeMAX() {
    pinMode(ALERT_PIN, INPUT_PULLDOWN);  // Use internal pull-down to ensure LOW by default
    pinMode(STATUS_LED_PIN, OUTPUT);     // Set status LED pin as output
    attachInterrupt(digitalPinToInterrupt(ALERT_PIN), alertISR, FALLING); // Trigger when it goes LOW
    //lipo.enableDebugging(); // Uncomment for debug messages
    lipo.quickStart();
    delay(100);
    if (!lipo.begin()) {
        return false;
        Serial.println(F("MAX17048 not detected. Check wiring."));
    }
    lipo.setThreshold(20);
    setVoltageAlertThresholds(MIN_BATT_VOLTAGE, MAX_BATT_VOLTAGE);
    delay(1000);
    quickStatusOnBlink();
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

//TODO integrate it with decision logic
//something like: if (!significantActivityDetected)
//                    1.  send false alarm
//                    2.  send batt readings
//                  else sendResults();

bool significantActivityDetected() {
    return (max_chainsaw_magnitude > max_machete_magnitude) ||
           (max_machete_magnitude > max_saw_magnitude) ||
           (overall_vibration_magnitude > VIBRATION_THRESHOLD);
}

void monitorFor60Sec() {
    unsigned long startTime = millis();
    monitorBattery();
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
        printResults();
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

//TODO Is this really taking effect before fft?
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

void printResults() {

/* meshtastic equivalent:

// Struct definitions
typedef struct _meshtastic_TreeShake {
    float max_axe_magnitude;
    float max_saw_magnitude;
    float max_chainsaw_magnitude;
    float max_machete_magnitude;
    bool alert;
} meshtastic_TreeShake;

// Packing messages
void readSensorsToPB(meshtastic_TreeShake & msg){
    msg.max_machete_magnitude = max_machete_magnitude;
    msg.max_chainsaw_magnitude = max_chainsaw_magnitude;
    msg.max_axe_magnitude = max_axe_magnitude;
    msg.max_saw_magnitude = max_saw_magnitude;
    msg.voltage = voltage;
    msg.alert = alert;
}
*/
    //TODO figure out a way to categorize these results via tresholds or SAMPLING_RATE
    Serial.println("===== Sensor Results =====");
    Serial.printf("Machete: %.2f\n", max_machete_magnitude);
    Serial.printf("Chainsaw: %.2f\n", max_chainsaw_magnitude);
    Serial.printf("Saw: %.2f\n", max_saw_magnitude);
    Serial.printf("Voltage: %.2fV\n", voltage);
    Serial.printf("Battery: %.2f%%\n", soc);
    Serial.println("=========================");

    //TODO package these 3 levels in a meshtastic message, maybe float?
    // Battery level warnings logic
    // if (voltage < 3.8 && voltage >= 3.3) {
    //     Serial.println(("⚠️ Battery Low:")(voltage)(" V")(". Consider Charging Soon."));
    // } else if (voltage < 3.3 && voltage >= 3.0) {
    //     Serial.println(("❗ Battery Critically Low:")(voltage)(" V")(". Charge Immediately."));
    // } else if (voltage < 3.0) {
    //     Serial.println(("🚨 Battery Dangerously Low:")(voltage)(" V")(". Stop Use & Charge Now!"));
    // } else if (alert) {
    // Serial.println("🔴 HIGH PULSE INTERRUPT DETECTED: LOW BATTERY ALERT!");

    //TODO unify blinking alerts and document
    // Blink LED 5 times quickly (100ms ON, 100ms OFF)
    for (int i = 0; i < 5; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
    }
    // Clear alert after displaying
    lipo.clearAlert();
    alert = false;
}