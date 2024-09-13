#include <Arduino.h>
#include "config.h"
#include "sensor.h"
#include "file_system.h"
#include "alarm.h"
#include "utilities.h"

RTC_DATA_ATTR int bootCount = 0;
volatile bool triggered = false;

SensorManager sensorManager;

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("Starting setup...");

    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));

    printWakeupReason();

    // Initialize GPIO7 as an interrupt source
    pinMode(LED_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, RISING);
    esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, HIGH);

    Wire.begin(SDA_PIN, SCL_PIN);

    initFileSystem();

    Serial.println("MPU6050 chip detected");

    bool tooMuchShake = false;
    int tooMuchShakeCount = 0;
    int consecutiveNoShakeCount = 0;

    for (int i = 0; i < 5; i++)
    {
        Serial.print("Starting sampling cycle ");
        Serial.println(i + 1);

        toggleAccelPower(true);
        delay(5000);

        Serial.println("Starting first sampling cycle");

        bool tooMuchShakeResult = sensorManager.inspectTreeShaking();
        toggleAccelPower(false);

        Serial.print("Sampling cycle ");
        Serial.print(i + 1);
        Serial.println(" completed");

        // handling shake detection logic
        if (tooMuchShakeResult)
        {
            sendAlarm();
        }

        if (i < 4)
        {
            Serial.println("Waiting for next sampling cycle...");
            delay(2000);
        }
    }

    Serial.println("Going to sleep zzz");
        // Re-enable GPIO7 as an interrupt source before sleeping
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    esp_deep_sleep_start();
}

void loop()
{
    // Not used
}
