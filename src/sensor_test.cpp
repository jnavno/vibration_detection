#include <Wire.h>
#include <MPU6050.h>   
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

// Define I2C Pins
#define SDA_PIN 41
#define SCL_PIN 42

// Define Control Pins
#define VEXT_CTRL_PIN 36
#define INT_PIN 7
#define STATUS_LED_PIN 4  
#define ALERT_LED_PIN 5  

MPU6050 mpu;
SFE_MAX1704X lipo;

// Interrupt tracking
volatile bool interruptOccurred = false;
volatile unsigned long lastInterruptTime = 0;
volatile unsigned long interruptDuration = 0;
volatile unsigned long disableTime = 0; // Stores when the interrupt was disabled
bool interruptEnabled = true; // Tracks if interrupt is enabled

// Function prototypes
bool testMPU();
void readMPU();
bool testMAX();
void readMAX();
void powerVEXT(bool state);
void IRAM_ATTR handleInterrupt();  // Interrupt Service Routine (ISR)
void checkInterruptQuality();
void restoreInterruptIfNeeded();

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);

    // Configure VEXT power pin
    pinMode(VEXT_CTRL_PIN, OUTPUT);
    powerVEXT(true);  // Power up sensors
    delay(100);  // Ensure power is stable

    // Configure interrupt pin
    pinMode(INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING);

    Serial.println("Starting sensor test...");

    // Test MPU6050
    if (testMPU()) {
        Serial.println("MPU6050 detected and initialized.");
    } else {
        Serial.println("ERROR: MPU6050 NOT detected!");
    }

    // Test MAX1704x
    if (testMAX()) {
        Serial.println("MAX1704x detected and initialized.");
    } else {
        Serial.println("ERROR: MAX1704x NOT detected!");
    }
}

void loop() {
    readMPU();
    readMAX();
    
    // Ensure we check interrupts continuously
    checkInterruptQuality();

    // Restore interrupt if enough time has passed
    restoreInterruptIfNeeded();

    delay(1500);  // 1 second delay for testing
}

// === FUNCTION DEFINITIONS ===

// Power Control
void powerVEXT(bool state) {
    if (state) {
        digitalWrite(VEXT_CTRL_PIN, LOW);
    } else {
        digitalWrite(VEXT_CTRL_PIN, HIGH);
    }
}

// Interrupt Service Routine (ISR) - Runs when GPIO7 receives a pulse
void IRAM_ATTR handleInterrupt() {
    unsigned long currentTime = millis();
    
    if (lastInterruptTime > 0) {
        interruptDuration = currentTime - lastInterruptTime;
    }
    
    lastInterruptTime = currentTime;
    interruptOccurred = true;

    // Disable interrupt for 10 seconds
    detachInterrupt(INT_PIN);
    disableTime = millis();
    interruptEnabled = false;
}

// Check the quality of the interrupt signal
void checkInterruptQuality() {
    static unsigned long lastCheckTime = 0;

    // Ensure we check every second even if no interrupt occurs
    if (millis() - lastCheckTime > 1000) {
        Serial.println("Waiting for interrupts on GPIO7...");
        lastCheckTime = millis();
    }

    if (interruptOccurred) {
        Serial.print("Interrupt detected on GPIO7 | ");
        Serial.print("Time since last pulse: ");
        Serial.print(interruptDuration);
        Serial.println(" ms");

        // Reset flag
        interruptOccurred = false;
    }
}

// Restore the interrupt after 10 seconds
void restoreInterruptIfNeeded() {
    if (!interruptEnabled && (millis() - disableTime >= 10000)) { // 10 seconds have passed
        Serial.println("Restoring interrupt functionality on GPIO7...");
        attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING);
        interruptEnabled = true;
    }
}

// MPU6050 Functions
bool testMPU() {
    mpu.initialize();
    return mpu.testConnection();
}

void readMPU() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Serial.print("MPU Data | Acc: ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.print(az); Serial.print(" | Gyro: ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.println(gz);
}

// MAX1704x Functions
bool testMAX() {
    return lipo.begin();
}

void readMAX() {
    Serial.print("Battery Voltage: ");
    Serial.print(lipo.getVoltage());
    Serial.print("V | Percentage: ");
    Serial.print(lipo.getSOC());
    Serial.println("%");
}
