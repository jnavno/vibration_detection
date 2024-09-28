#include "powerManager.h"
#include <Arduino.h>
#include <variant.h>

void setupPower() {
    pinMode(ACCEL_PWR_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
}

void manageBattery() {
    // Add battery monitoring logic here (if needed)
}

void toggleAccelPower(bool state) {
    digitalWrite(ACCEL_PWR_PIN, state ? LOW : HIGH);
}

void quickBlinkAndHalt() {
    Serial.println("5-minute data logging limit reached. Press reset to restart.");

    unsigned long blinkStart = millis();
    while (millis() - blinkStart < 30000) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(200);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(200);
    }

    Serial.println("Data logging halted. Press reset to restart.");
    while (true);
}
