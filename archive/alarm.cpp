#include "alarm.h"
#include <Arduino.h>

void sendAlarm() {
    pinMode(LED_PIN, OUTPUT); // Ensure LED pin is set as output
    for (int i = 0; i < 12; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
    Serial.println("Alarm sent: Strong Tree shaking detected. Potential threat at x,y coordinates");
}