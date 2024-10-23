#include "powerManager.h"
#include <Arduino.h>
#include <variant.h>

void setupPower() {
    pinMode(VEXT_CTRL_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);

    digitalWrite(VEXT_CTRL_PIN, HIGH);
}

void manageBattery() {
    // Add battery monitoring logic here (if needed)
}

void toggleSensorPower(bool state) {
    if (state) {
        digitalWrite(VEXT_CTRL_PIN, LOW);  // Vext ON
        delay(PRE_TOGGLE_DELAY);           // Wait for the sensor to stabilize
    } else {
        digitalWrite(VEXT_CTRL_PIN, HIGH); // Vext OFF
        delay(300);                        // Allow some time for sensor to power down
    }
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
