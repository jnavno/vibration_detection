#pragma once
#include "powerManager.h"
#include <Arduino.h>
#include <variant.h>

void setupPower();
void manageBattery();
void toggleSensorPower(bool state);
void quickBlinkAndHalt();

void setupPower() {
    pinMode(VEXT_CTRL_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(VEXT_CTRL_PIN, HIGH);
}

void manageBattery() {
    // Placeholder for battery management logic
}

void toggleSensorPower(bool state) {
    if (state) {
        digitalWrite(VEXT_CTRL_PIN, LOW);  // Vext ON
        delay(PRE_TOGGLE_DELAY);
    } else {
        digitalWrite(VEXT_CTRL_PIN, HIGH); // Vext OFF
        delay(300);
    }
}

void quickBlinkAndHalt() {
    Serial.println("System halted. Press reset to restart.");
    unsigned long blinkStart = millis();
    while (millis() - blinkStart < 30000) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(200);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(200);
    }
    while (true);
}