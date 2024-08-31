#include <Arduino.h>

const int blinkPin = 7;  // GPIO 7

void setup() {
    pinMode(blinkPin, OUTPUT);  // Set GPIO 7 as an output
}

void loop() {
    digitalWrite(blinkPin, HIGH);  // Turn the LED on
    delay(1000);                   // Wait for a second
    digitalWrite(blinkPin, LOW);   // Turn the LED off
    delay(1000);                   // Wait for a second
}
