#include <Wire.h>
#include <MPU6050.h>   
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include "esp_sleep.h"
#include "esp_bt.h"
#include "driver/adc.h"
#include <DebugConfiguration.h>
#include "variant.h"
#include "driver/rtc_io.h"

#define WAKEUP_TIME_US 259200000000ULL  // Deep Sleep Interval (72 hours)
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  259200
MPU6050 mpu;
SFE_MAX1704X lipo;

// **Function Prototypes**
void powerVEXT(bool state);
bool testMPU();
bool testMAX();
void enterDeepSleep();
void disablePeripherals();
void disableWakeInterrupt();
void enableWakeInterrupt();

void setup() {
    INIT_DEBUG_SERIAL();
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

    LOG_DEBUGLN("Reading Sensor Data...");
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    delay(100);
    float voltage = lipo.getVoltage();
    float soc = lipo.getSOC();

    // **Print JSON Data**
    LOG_DEBUGLN("=== SENSOR DATA ===");
    Serial.print("{\"accel_x\": "); Serial.print(ax);
    Serial.print(", \"accel_y\": "); Serial.print(ay);
    Serial.print(", \"accel_z\": "); Serial.print(az);
    Serial.print(", \"gyro_x\": "); Serial.print(gx);
    Serial.print(", \"gyro_y\": "); Serial.print(gy);
    Serial.print(", \"gyro_z\": "); Serial.print(gz);
    Serial.print(", \"battery_v\": "); Serial.print(voltage, 2);
    Serial.print(", \"battery_soc\": "); Serial.print(soc, 1);
    Serial.println("}");

    for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(200);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(200);
    }

    enterDeepSleep();
}

// **Power Control**
void powerVEXT(bool state) {
    digitalWrite(VEXT_CTRL_PIN, state ? LOW : HIGH);
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

void prepare_for_deep_sleep() {
    LOG_DEBUGLN("Preparing for deep sleep...");

    // RTC GPIOs to be isolated
    gpio_num_t rtc_pins_to_isolate[] = {
        GPIO_NUM_1, GPIO_NUM_2
    };

    // Non-RTC GPIOs set to low-power state (input with pull-down)
    gpio_num_t non_rtc_pins[] = {
        GPIO_NUM_19, GPIO_NUM_20, GPIO_NUM_21, GPIO_NUM_26,
        GPIO_NUM_33, GPIO_NUM_34, GPIO_NUM_35, GPIO_NUM_36,
        GPIO_NUM_43, GPIO_NUM_44
    };

    // General unused GPIOs to prevent floating states
    gpio_num_t unused_pins[] = {
        GPIO_NUM_47, GPIO_NUM_48
    };

    // Isolate RTC GPIOs
    LOG_DEBUGLN("Isolating RTC GPIOs to prevent leakage...");
    for (size_t i = 0; i < sizeof(rtc_pins_to_isolate) / sizeof(rtc_pins_to_isolate[0]); i++) {
        if (rtc_gpio_isolate(rtc_pins_to_isolate[i]) != ESP_OK) {
            LOG_DEBUG("Error: Failed to isolate RTC GPIO %d", rtc_pins_to_isolate[i]);
        } else {
            LOG_DEBUG("RTC GPIO %d isolated.", rtc_pins_to_isolate[i]);
        }
    }

    // Setting non-RTC GPIOs as input with pull-down
    LOG_DEBUGLN("Configuring non-RTC GPIOs for low power...");
    for (size_t i = 0; i < sizeof(non_rtc_pins) / sizeof(non_rtc_pins[0]); i++) {
        if (gpio_set_direction(non_rtc_pins[i], GPIO_MODE_INPUT) != ESP_OK) {
            LOG_DEBUG("Error: Failed to set GPIO %d as input.", non_rtc_pins[i]);
        }
        if (gpio_pulldown_en(non_rtc_pins[i]) != ESP_OK) {
            LOG_DEBUG("Error: Failed to enable pull-down on GPIO %d.", non_rtc_pins[i]);
        }
        if (gpio_hold_en(non_rtc_pins[i]) != ESP_OK) {
            LOG_DEBUG("Error: Failed to enable hold on GPIO %d.", non_rtc_pins[i]);
        } else {
            LOG_DEBUG("GPIO %d set to input with pull-down and hold enabled.", non_rtc_pins[i]);
        }
    }

    // Configuring general unused GPIOs
    LOG_DEBUGLN("Configuring unused GPIOs to prevent floating states...");
    for (size_t i = 0; i < sizeof(unused_pins) / sizeof(unused_pins[0]); i++) {
        if (gpio_set_direction(unused_pins[i], GPIO_MODE_INPUT) != ESP_OK) {
            LOG_DEBUG("Error: Failed to set GPIO %d as input.", unused_pins[i]);
        }
        if (gpio_pulldown_en(unused_pins[i]) != ESP_OK) {
            LOG_DEBUG("Error: Failed to enable pull-down on GPIO %d.", unused_pins[i]);
        }
        if (gpio_hold_en(unused_pins[i]) != ESP_OK) {
            LOG_DEBUG("Error: Failed to enable hold on GPIO %d.", unused_pins[i]);
        } else {
            LOG_DEBUG("Unused GPIO %d set to input with pull-down and hold enabled.", unused_pins[i]);
        }
    }

    // Keeping RTC Peripheral Power ON, required for wake-up sources
    LOG_DEBUGLN("Keeping RTC Peripheral Power ON...");
    if (esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON) != ESP_OK) {
        LOG_DEBUGLN("Error: Failed to configure RTC power domain.");
    }

    LOG_DEBUGLN("System prepared for deep sleep.");
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
    // prepare_for_deep_sleep();

    LOG_DEBUGLN("Entering deep sleep...");
    delay(100);
    Serial.flush();
    delay(50);

    esp_deep_sleep_start();
}

bool testMPU() {
    LOG_DEBUGLN("Initializing MPU6050...");

    mpu.initialize();
    if (!mpu.testConnection()) {
        LOG_DEBUGLN("ERROR: MPU6050 NOT detected!");
        return false;
    }

    // MPU6050 might be in sleep mode, this will wake it up.
    mpu.setSleepEnabled(false);
    delay(100);

    LOG_DEBUGLN("MPU6050 initialized successfully.");
    return true;
}

// **MAX1704x Battery Sensor Initialization**
bool testMAX() {
    return lipo.begin();
}

void loop() {
    // **Empty because ESP32 goes to sleep in setup()**
}
