/*
/////////// BASIC SENSOR TESTING MODULE //////////////////
///////////   MPU6050 & MAX1704X READINGS  ///////////////

- DUAL wakeup turns on device (TIMER or EXT_0)
- Device just outputs accel and battery readings automatically allocated from the stack
- Accel data is collected at 1000Hz to fit training classifier purposes
- Readings ignore gyroscope data due to negligible angular rotation for this use case
- BLE is disabled to save power
- INT from EXT_0 is disabled during operation
- INT re-enabled before entering deep sleep

============================================================
🛠️ Developer Quick Checklist (Before Flashing)
------------------------------------------------------------
- Verify correct GPIO assignments in `variant.h`:
    - VEXT_CTRL_PIN (sensor power)
    - SDA_PIN, SCL_PIN (I2C)
    - INTERRUPT_PIN (wake trigger)
- Ensure a battery is connected to the hat's BAT socket, for VEXT power
- Confirm Serial Monitor baud rate is set to 115200
- Confirm PlatformIO `.ini` board matches ESP32S3/Heltec V3 hardware
- Confirm BLE is disabled automatically (saves power)

============================================================
🚀 How to Flash + Monitor
------------------------------------------------------------
1. Connect device via USB
2. Flash firmware:
    PlatformIO:   `platformio run -t upload`
    Arduino IDE:  Upload as usual
3. Open Serial Monitor at 115200 baud:
    PlatformIO:   `platformio device monitor -b 115200`
    Arduino IDE:  Tools > Serial Monitor
4. Confirm:
    - Sensor initialization prints to console
    - JSON-formatted sensor readings appear
    - Status LED blinks 3x after reading
    - Device enters deep sleep
5. To re-test: press reset button or trigger EXT_0 INT.

============================================================
*/


#include <Wire.h>
#include <MPU6050.h>   
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include "esp_sleep.h"
#include "esp_bt.h"
#include "driver/adc.h"
#include <DebugConfiguration.h>
#include "variant.h"
#include "driver/rtc_io.h"

#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP  259200   /*72h sleeping*/
MPU6050 mpu;
SFE_MAX1704X lipo;

void powerVEXT(bool state);
bool testMPU();
bool testMAX();
void enterDeepSleep();
void disablePeripherals();
void disableWakeInterrupt();
void enableWakeInterrupt();
void readSensorData();

void setup() {
    INIT_DEBUG_SERIAL();
    LOG_DEBUG("========== SENSOR TEST ==========");

    LOG_DEBUGLN("Booting...");

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    LOG_DEBUG("Wake-up reason: "); LOG_DEBUGLN(wakeup_reason);

    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        LOG_DEBUGLN("Woke up from GPIO7 (Mechanical Sensor Trigger)");
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        LOG_DEBUGLN("Woke up from Timer (72h)");
    } else {
        LOG_DEBUGLN("Cold Boot (Power-on Reset)");
    }
    disableWakeInterrupt();
    disablePeripherals();

    pinMode(VEXT_CTRL_PIN, OUTPUT);
    powerVEXT(true);
    LOG_DEBUGLN("Sensors initialized.");
    delay(100);

    Wire.begin(SDA_PIN, SCL_PIN);
    LOG_DEBUGLN("I2C reinitialized.");

    bool mpuOK = testMPU();
    if (!mpuOK) LOG_DEBUGLN("ERROR: MPU6050 NOT detected!");

    bool maxOK = testMAX();
    if (!maxOK) LOG_DEBUGLN("ERROR: MAX1704x NOT detected!");

    readSensorData();

    for (int i = 0; i < 3; i++) {   /*visual output: SETUP DONE*/
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(200);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(200);
    }

    enterDeepSleep();
}

void powerVEXT(bool state) {
    digitalWrite(VEXT_CTRL_PIN, state ? LOW : HIGH);
}

void readSensorData() {
    LOG_DEBUGLN("Reading Sensor Data...");
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    delay(100);
    float voltage = lipo.getVoltage();
    float soc = lipo.getSOC();

    LOG_DEBUGLN("=== SENSOR DATA ===");
    Serial.print("{\"accel_x\": "); Serial.print(ax);
    Serial.print(", \"accel_y\": "); Serial.print(ay);
    Serial.print(", \"accel_z\": "); Serial.print(az);
    Serial.print(", \"battery_v\": "); Serial.print(voltage, 2);
    Serial.print(", \"battery_soc\": "); Serial.print(soc, 1);
    Serial.println("}");

}

void disableWakeInterrupt() {
    LOG_DEBUGLN("Disabling all wake-up sources...");

    esp_err_t err = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    if (err == ESP_OK) {
        LOG_DEBUGLN("All wake-up sources disabled successfully.");
    } else {
        LOG_DEBUGLN("ERROR: Failed to disable one or more wake-up sources.");
    }
}


void enableWakeInterrupt() {
    LOG_DEBUGLN("Re-arming INT for next wake-up...");
        LOG_DEBUG("INTERRUPT_PIN = "); LOG_DEBUGLN(INTERRUPT_PIN);
    pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);

    esp_err_t errI = esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, 1);
    rtc_gpio_pullup_dis(INTERRUPT_PIN);
    rtc_gpio_pulldown_en(INTERRUPT_PIN);
    esp_err_t errT = esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

    if (errI == ESP_OK && errT == ESP_OK) {
        LOG_DEBUGLN("All wake-up sources enabled successfully.");
    } else {
        LOG_DEBUGLN("ERROR: Failed to enable one or more wake-up sources.");
    }
}


void disablePeripherals() {
    LOG_DEBUGLN("Disabling unused peripherals...");


    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        LOG_DEBUG("Disabling Bluetooth...");
        esp_err_t btErr = esp_bt_controller_disable();
        if (btErr == ESP_OK) {
            LOG_DEBUGLN("Bluetooth successfully disabled.");
        } else {
            LOG_DEBUG("ERROR: Failed to disable Bluetooth. Error code: %d", btErr);
        }
    } else {
        LOG_DEBUGLN("Bluetooth was already disabled.");
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


bool testMAX() {
    LOG_DEBUGLN("Initializing MAX17048...");

    if (!lipo.begin()) {
        LOG_DEBUGLN("ERROR: MAX17048 NOT detected!");
        return false;
    }
    lipo.quickStart(); // Force chip to reset SoC calculation
    delay(200);

    LOG_DEBUGLN("MAX17048 initialized successfully.");
    return true;
}
void loop() {
}
