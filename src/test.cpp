#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "SPIFFS.h"

#define LED_PIN 3                // Define the LED pin (GPIO3)
#define ACCEL_PWR_PIN 5          // Define the pin to control the MOSFET
#define QUICK_BLINK_DELAY 100    // Quick blink delay in milliseconds
#define SLOW_BLINK_DELAY 1000    // Slow blink delay in milliseconds
#define DATA_STREAM_DURATION 5000  // Duration for data streaming in milliseconds
#define NO_DATA_STREAM_DURATION 5000  // Duration for no data streaming in milliseconds
#define PRE_TOGGLE_DELAY 2000    // Delay before toggling accel power
#define INIT_DELAY 500           // Delay after powering on the accelerometer

Adafruit_MPU6050 mpu;

// Function prototypes
void blinkLED(int delayTime);
void readAccelerometer();
void toggleAccelPower(bool state);
bool initializeMPU();

void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);  // Set LED pin as output
    pinMode(ACCEL_PWR_PIN, OUTPUT);
    delay(2000);

    int count = 0;

    // Power on the accelerometer
    toggleAccelPower(true);
    delay(PRE_TOGGLE_DELAY);  // Allow time for accelerometer to power up

    if (!initializeMPU())
    {
        Serial.println("Failed to initialize MPU6050");
        while (1);
    }
    Serial.println("MPU6050 detected and initialized!");

    // Run the test loop indefinitely
    while (true)
    {
        // Data stream period
        unsigned long streamStartTime = millis();
        while (millis() - streamStartTime < DATA_STREAM_DURATION)
        {
            blinkLED(QUICK_BLINK_DELAY);  // Blink LED quickly
            readAccelerometer();
        }

        // No data stream period
        unsigned long noStreamStartTime = millis();
        while (millis() - noStreamStartTime < NO_DATA_STREAM_DURATION)
        {
            blinkLED(SLOW_BLINK_DELAY);  // Blink LED slowly
            delay(SLOW_BLINK_DELAY);     // Keep the LED off for the remaining time
        }

        // Turn off the accelerometer
        toggleAccelPower(false);
        delay(PRE_TOGGLE_DELAY);  // Allow time for the accelerometer to power down

        // Increment and print the cycle count
        Serial.print("Cycle: ");
        Serial.println(count++);
        
        // Power on the accelerometer for the next cycle
        toggleAccelPower(true);
        delay(PRE_TOGGLE_DELAY);  // Allow time for accelerometer to power up

        if (!initializeMPU())
        {
            Serial.println("Failed to reinitialize MPU6050");
            while (1);
        }
    }
}

void loop()
{
    // Not used
}

void blinkLED(int delayTime)
{
    digitalWrite(LED_PIN, HIGH);  // Turn LED on
    delay(delayTime);             // Wait for a short period
    digitalWrite(LED_PIN, LOW);   // Turn LED off
    delay(delayTime);             // Wait for a short period
}

void toggleAccelPower(bool state)
{
    digitalWrite(ACCEL_PWR_PIN, state ? HIGH : LOW);
}

bool initializeMPU()
{
    // Initialize MPU6050 and check if it's connected
    if (!mpu.begin())
    {
        return false;
    }
    delay(INIT_DELAY);  // Allow time for the MPU to settle
    return true;
}

void readAccelerometer()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Print accelerometer data
    Serial.print("X: ");
    Serial.print(a.acceleration.x);
    Serial.print(" m/s^2 ");
    Serial.print("Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(" m/s^2 ");
    Serial.print("Z: ");
    Serial.print(a.acceleration.z);
    Serial.print(" m/s^2 ");
    Serial.println();
}
