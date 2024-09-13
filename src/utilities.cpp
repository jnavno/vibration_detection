#include "utilities.h"
#include "config.h"

void printWakeupReason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            Serial.println("Wakeup caused by external signal using RTC_IO");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            Serial.println("Wakeup caused by external signal using RTC_CNTL");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("Wakeup caused by timer");
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            Serial.println("Wakeup caused by touchpad");
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            Serial.println("Wakeup caused by ULP program");
            break;
        default:
            Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
            break;
    }
}

void handleInterrupt() {
    extern volatile bool triggered;
    triggered = true;
}

void toggleAccelPower(bool state) {
    pinMode(ACCEL_POWER_PIN, OUTPUT);
    if (state) {
        Serial.println("Turning on accelerometer...");
        digitalWrite(ACCEL_POWER_PIN, HIGH); // Power on the accelerometer
        delay(2000);
        // No need to set pinMode to INPUT here as it may not be necessary.
    } else {
        Serial.println("Turning off accelerometer...");
        digitalWrite(ACCEL_POWER_PIN, LOW); // Power off the accelerometer
        delay(2000);
        // No need to set pinMode to INPUT here as it may not be necessary.
    }
}