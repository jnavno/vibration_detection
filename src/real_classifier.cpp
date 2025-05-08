// real_classifier.cpp
// ===== REAL-TIME CLASSIFICATION MODULE =====
// - Collects accelerometer data from MPU6050
// - Uses MAX1704x for battery voltage/SOC
// - Performs vibration classification in real time
// - Stores results in LittleFS and prints to Serial
// - Deep sleep between wake-ups (via INT pin or timer)

#include <Wire.h>
#include <MPU6050.h>
#ifdef USE_MAX1704X
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#endif
#include <FS.h>
#include <LittleFS.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <DebugConfiguration.h>
#include "variant.h"

#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP  86400  // 24h sleep fallback
#define CLASSIFICATION_FILE "/classification_log.txt"
#define USE_PLACEHOLDER_LOGIC

MPU6050 mpu;
#ifdef USE_MAX1704X
SFE_MAX1704X lipo;
#endif
bool max1704xPresent = false;

void powerVEXT(bool state);
bool testMPU();
bool testMAX();
void disableGyro();
void enableWakeInterrupt();
void disableWakeInterrupt();
void enterDeepSleep();
void classifyAndStore();

void setup() {
    INIT_DEBUG_SERIAL();
    LOG_DEBUGLN("Booting real-time classifier...");

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    Serial.print("Wakeup cause: ");
    Serial.println((int)wakeup_reason);

    pinMode(VEXT_CTRL_PIN, OUTPUT);
    powerVEXT(true);
    delay(200);

    Wire.begin(SDA_PIN, SCL_PIN);
    LOG_DEBUGLN("I2C initialized.");

    bool mpuOK = testMPU();
    if (!mpuOK) LOG_DEBUGLN("[ERROR] MPU6050 NOT detected!");
    else disableGyro();

    max1704xPresent = testMAX();
    if (!max1704xPresent) LOG_DEBUGLN("[WARNING] MAX1704x NOT detected.");

    if (!LittleFS.begin(true)) {
        LOG_DEBUGLN("[ERROR] LittleFS mount failed!");
    } else {
        LOG_DEBUGLN("[✓] LittleFS mounted.");
        File file = LittleFS.open(CLASSIFICATION_FILE, FILE_READ);
        if (file) {
            Serial.println("--- Classification Log ---");
            while (file.available()) Serial.write(file.read());
            file.close();
            Serial.println("--------------------------");
        }
    }

    classifyAndStore();
    delay(500);
    enterDeepSleep();
}

void loop() {}

void powerVEXT(bool state) {
    digitalWrite(VEXT_CTRL_PIN, state ? LOW : HIGH);
}

bool verifyGyroDisabled(bool verbose = true) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    bool gyroDisabled = (gx == 0 && gy == 0 && gz == 0);

    if (verbose) {
        if (gyroDisabled) {
            LOG_DEBUGLN("Gyroscope successfully disabled.");
        } else {
            LOG_DEBUGLN("WARNING: Gyroscope still active!");
        }
    }

    return gyroDisabled;
}

bool testMPU() {
    LOG_DEBUGLN("Initializing MPU6050...");

    mpu.initialize();
    if (!mpu.testConnection()) {
        LOG_DEBUGLN("ERROR: MPU6050 NOT detected!");
        return false;
    }

    mpu.setSleepEnabled(false); /*Prevents MPU to remain asleep after wakeup*/
    delay(100);
        LOG_DEBUGLN("Disabling Gyroscope...");
        mpu.setStandbyXGyroEnabled(true);
        mpu.setStandbyYGyroEnabled(true);
        mpu.setStandbyZGyroEnabled(true);
        delay(10);
        if (!verifyGyroDisabled()) {
            LOG_DEBUGLN("ERROR: Gyroscope not properly disabled! Aborting.");
            return false;
        }

        LOG_DEBUGLN("MPU6050 gyroscope disabled successfully.");
        LOG_DEBUGLN("MPU6050 initialized successfully.");
        return true;
}

void disableGyro() {
    mpu.setSleepEnabled(false);
    delay(100);
    mpu.setStandbyXGyroEnabled(true);
    mpu.setStandbyYGyroEnabled(true);
    mpu.setStandbyZGyroEnabled(true);
    LOG_DEBUGLN("Gyroscope disabled.");
}

bool testMAX() {
    #ifdef USE_MAX1704X
      LOG_DEBUGLN("Initializing MAX17048...");
      if (!lipo.begin()) {
        LOG_DEBUGLN("[MAX1704x] begin() failed — sensor not detected.");
        return false;
      }
      delay(200);
      Wire.beginTransmission(0x36);
      Wire.write(0x08);
      Wire.endTransmission(false);
      Wire.requestFrom(0x36, 2);
      uint16_t version = (Wire.read() << 8) | Wire.read();
      LOG_DEBUG("[MAX1704x] Version: 0x%04X\n", version);
    
      Wire.beginTransmission(0x36);
      Wire.write(0x0C);
      Wire.endTransmission(false);
      Wire.requestFrom(0x36, 2);
      uint16_t config = (Wire.read() << 8) | Wire.read();
      bool sleeping = config & (1 << 7);
      LOG_DEBUG("[MAX1704x] CONFIG: 0x%04X — Sleeping: %s\n", config, sleeping ? "Yes" : "No");
    
      Wire.beginTransmission(0x36);
      Wire.write(0xFE);
      Wire.write(0x00);
      Wire.write(0x54);
      Wire.endTransmission();
      LOG_DEBUGLN("[MAX1704x] Reset command sent.");
    
      delay(1000);
      if (!lipo.begin()) {
        LOG_DEBUGLN("[MAX1704x] Re-init after reset failed.");
        return false;
      }
    
      LOG_DEBUGLN("[MAX1704x] Monitoring initial SOC...");
      for (int i = 0; i < 5; i++) {
        float voltage = lipo.getVoltage();
        float soc = lipo.getSOC();
        LOG_DEBUG("[MAX1704x] Voltage: %.2f V, SOC: %.2f %%\n", voltage, soc);
        delay(1000);
      }
      return true;
    #else
      return false;
    #endif
    }
    void classifyAndStore() {
        LOG_DEBUGLN("[INFO] Reading sensors and classifying...");
    
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
        float accel_mag = sqrt(ax * ax + ay * ay + az * az);
        float tempC = mpu.getTemperature() / 340.00 + 36.53;
        float voltage = max1704xPresent ? lipo.getVoltage() : -1.0f;
        float soc = max1704xPresent ? lipo.getSOC() : -1.0f;
    
        String result = "⚠️ Unclassified";
    #ifdef USE_PLACEHOLDER_LOGIC
        if (accel_mag > 25000 && ay > 15000 && az > 15000) {
            result = "✅ Likely Machete";
        } else if (accel_mag > 15000 && soc > 20.0f) {
            result = "🔧 Possibly Chainsaw";
        }
    #endif
    
        Serial.printf("Classification: %s\n", result.c_str());
    
        File file = LittleFS.open(CLASSIFICATION_FILE, FILE_APPEND);
        if (file) {
            file.printf("%lu,%d,%d,%d,%.2f,%.2f,%.2f,%s\n",
                        millis(), ax, ay, az, accel_mag, voltage, soc, result.c_str());
            file.close();
            LOG_DEBUGLN("[✓] Result stored in LittleFS.");
        } else {
            LOG_DEBUGLN("[ERROR] Could not write to LittleFS.");
        }
    }

void enterDeepSleep() {
    LOG_DEBUGLN("Preparing for deep sleep...");

    LOG_DEBUGLN("Setting GPIO7 correctly before sleep...");
    pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);
    LOG_DEBUG("GPIO7 State Before Sleep: "); LOG_DEBUGLN(digitalRead(INTERRUPT_PIN));

    LOG_DEBUGLN("Disabling I2C...");
    Wire.end();

    LOG_DEBUGLN("Disabling sensors...");
    powerVEXT(false);
    delay(50);

    enableWakeInterrupt();

    LOG_DEBUGLN("Entering deep sleep...");
    delay(100);
    Serial.flush();
    delay(200);

    esp_deep_sleep_start();
}
void enableWakeInterrupt() {
    pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);
    rtc_gpio_pullup_dis((gpio_num_t)INTERRUPT_PIN);
    rtc_gpio_pulldown_en((gpio_num_t)INTERRUPT_PIN);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)INTERRUPT_PIN, 1);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void disableWakeInterrupt() {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
}
