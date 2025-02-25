#include "powerManager.h"
#include "DebugConfiguration.h"
#include <Arduino.h>
#include <variant.h>
#include <Wire.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

void setupPower();
void monitorBattery();
void toggleSensorPower(bool state);
void quickBlinkAndHalt();


void setupPower() {
    pinMode(VEXT_CTRL_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(VEXT_CTRL_PIN, HIGH);
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
    debug_println("System halted. System will restart.");
    unsigned long blinkStart = millis();
    // Blink pattern: two quick flashes with a longer pause (repeat for 30 seconds)
    while (millis() - blinkStart < 30000) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(700);
    }
    // Instead of halting with an infinite loop, restart the system.
    esp_restart();
}

//  Successful sensor initialization signaling
void SensorInitOKBlink() {
    debug_println("Successful sensor initialization indicated via LED");
    for (int i = 0; i < 5; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
    }
}