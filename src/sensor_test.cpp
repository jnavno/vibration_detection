#include <Wire.h>
#include <MPU6050.h>   
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include "esp_sleep.h"

// Define I2C Pins
#define SDA_PIN 41
#define SCL_PIN 42

// Define Control Pins
#define VEXT_CTRL_PIN 36
#define INT_PIN 7  // GPIO7 - Interrupt / ADC Pin
#define STATUS_LED_PIN 4  
#define ALERT_LED_PIN 5  

// Uncomment to enable ADC testing (disable interrupts)
//#define ENABLE_ADC_TESTING

// Deep Sleep Interval (72 hours)
#define WAKEUP_TIME_US 259200000000ULL  // 72 hours in microseconds

MPU6050 mpu;
SFE_MAX1704X lipo;

// **Function Prototypes (Added to Fix Errors)**
void powerVEXT(bool state);
bool testMPU();
bool testMAX();
void enableInterrupt();
void enterDeepSleep();
void IRAM_ATTR handleInterrupt();


void setup() {
    Serial.begin(115200);
    Serial.println("Booting...");

    // Detect wake-up cause
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("Woke up from GPIO7 (Mechanical Sensor Trigger)");
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println("Woke up from 72-hour timer");
    } else {
        Serial.println("Cold boot (Power-on Reset)");
    }

    // Initialize I2C and Power Up Sensors
    Wire.begin(SDA_PIN, SCL_PIN);
    pinMode(VEXT_CTRL_PIN, OUTPUT);
    powerVEXT(true);  // Turn on external power
    delay(100);

    #ifdef ENABLE_ADC_TESTING
        Serial.println("ADC mode active. Measuring voltage on GPIO7...");
        pinMode(INT_PIN, INPUT);  // Set GPIO7 as ADC input
    #else
        Serial.println("Interrupt mode active.");
        detachInterrupt(digitalPinToInterrupt(INT_PIN));  // Prevent unwanted wake-ups
    #endif

    // Initialize MPU6050
    bool mpuOK = testMPU();
    if (mpuOK) {
        Serial.println("MPU6050 detected and initialized.");
    } else {
        Serial.println("ERROR: MPU6050 NOT detected!");
    }

    // Initialize MAX1704x (Battery Fuel Gauge)
    bool maxOK = testMAX();
    if (maxOK) {
        Serial.println("MAX1704x detected and initialized.");
    } else {
        Serial.println("ERROR: MAX1704x NOT detected!");
    }

    // Read Sensor Data
    Serial.println("Reading Sensor Data...");
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float voltage = lipo.getVoltage();
    float soc = lipo.getSOC();

    // Print data in Meshtastic JSON format
    Serial.println("=== SENSOR DATA (Meshtastic Format) ===");
    Serial.print("{\"accel_x\": "); Serial.print(ax);
    Serial.print(", \"accel_y\": "); Serial.print(ay);
    Serial.print(", \"accel_z\": "); Serial.print(az);
    Serial.print(", \"gyro_x\": "); Serial.print(gx);
    Serial.print(", \"gyro_y\": "); Serial.print(gy);
    Serial.print(", \"gyro_z\": "); Serial.print(gz);
    Serial.print(", \"battery_v\": "); Serial.print(voltage, 2);
    Serial.print(", \"battery_soc\": "); Serial.print(soc, 1);
    #ifdef ENABLE_ADC_TESTING
        float adcVoltage = readADC();
        Serial.print(", \"adc_v\": "); Serial.print(adcVoltage, 3);
    #endif
    Serial.println("}");

    // Blink LED 3 times before sleep
    for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(200);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(200);
    }

    // Re-enable GPIO7 interrupt before sleep (if not in ADC mode)
    #ifndef ENABLE_ADC_TESTING
        enableInterrupt();
    #endif

    // Enter Deep Sleep
    enterDeepSleep();
}

// === FUNCTION DEFINITIONS ===

// Power Control
void powerVEXT(bool state) {
    digitalWrite(VEXT_CTRL_PIN, state ? LOW : HIGH);
}

// ADC Measurement on GPIO7
#ifdef ENABLE_ADC_TESTING
float readADC() {
    int rawValue = analogRead(INT_PIN);
    return (rawValue / 4095.0) * 3.3;  // Convert raw ADC value to voltage
}
#endif

// Enable Interrupt on GPIO7
void enableInterrupt() {
    pinMode(INT_PIN, INPUT_PULLDOWN);  // Prevents floating pin issues
    attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING);
    Serial.println("Interrupt on GPIO7 re-enabled.");
}

// Interrupt Service Routine (ISR)
void IRAM_ATTR handleInterrupt() {
    detachInterrupt(digitalPinToInterrupt(INT_PIN));  // Disable interrupt immediately
}

// Deep Sleep Function
void enterDeepSleep() {
    Serial.println("Configuring wake-up sources...");

    #ifndef ENABLE_ADC_TESTING
        // Enable GPIO wake-up on GPIO7 (Mechanical Sensor)
        esp_sleep_enable_ext0_wakeup((gpio_num_t)INT_PIN, 1);  // Wake when GPIO7 goes HIGH
    #endif

    // Enable Timer wake-up (72 hours)
    esp_sleep_enable_timer_wakeup(WAKEUP_TIME_US);

    Serial.println("Entering Deep Sleep...");
    delay(100);  // Ensure serial messages are sent before sleeping
    esp_deep_sleep_start();  // Put ESP32 into deep sleep
}

// MPU6050 Functions
bool testMPU() {
    mpu.initialize();
    return mpu.testConnection();
}

// MAX1704x Functions
bool testMAX() {
    return lipo.begin();
}

void loop() {
    // Empty because ESP32 goes to sleep in setup()
}