#include "LEDStatus.h"
#include "variant.h"
#include <Arduino.h>

/*

/////////// LED FEEDBACK LEGEND //////////////////

Purpose                     LED     Pattern               Meaning
---------------------------------------------------------------------
✅ SD card initialized      STATUS  3 short blinks        System ready to log
🔴 SD init failed           ALERT   3 slow blinks         Error condition, system restarting
🟨 Each 12s block finished  STATUS  1 very quick blink    Block saved to microSD
❌ Sensor not detected      ALERT   5 short blinks        Warning: sensor problem (MPU or MAX)
✅ Final completion         STATUS  3 slow blinks         All blocks complete
---------------------------------------------------------------------
*/

void blinkLED(int pin, int blinks, int on_ms, int off_ms) {
    for (int i = 0; i < blinks; i++) {
        digitalWrite(pin, HIGH);
        delay(on_ms);
        digitalWrite(pin, LOW);
        delay(off_ms);
    }
}

void blinkStatusShort()    { blinkLED(STATUS_LED_PIN, 3, 100, 100); }
void blinkStatusQuick()    { blinkLED(STATUS_LED_PIN, 1, 40, 40); }
void blinkStatusSlow()     { blinkLED(STATUS_LED_PIN, 3, 300, 200); }
void blinkAlertError()     { blinkLED(ALERT_LED_PIN, 3, 300, 300); }
void blinkAlertSensor()    { blinkLED(ALERT_LED_PIN, 5, 100, 100); }
void blinkStatusOK()    { blinkLED(STATUS_LED_PIN, 3, 50, 50); }     // Faster OK blink
void blinkAlertPulse()  { blinkLED(ALERT_LED_PIN, 1, 30, 200); }     // Plane-wing pulse blink
