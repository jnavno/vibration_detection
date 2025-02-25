#include "variant.h"
#include <Arduino.h>
#include "DebugConfiguration.h"

void sendAlarm() {
    pinMode(ALERT_LED_PIN, OUTPUT); // Ensure LED pin is set as output
    for (int i = 0; i < 12; i++) {
        digitalWrite(ALERT_LED_PIN, HIGH);
        delay(100);
        digitalWrite(ALERT_LED_PIN, LOW);
        delay(100);
    }
    debug_println("Alarm sent: Strong Tree shaking detected. Potential threat at x,y coordinates");
}