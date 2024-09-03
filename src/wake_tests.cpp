#include <Arduino.h>
#include "esp_sleep.h"
#include "config.h"
#include "sensor.h"
#include "file_system.h"
#include "alarm.h"
#include "utilities.h"

// Define the GPIO pins
#define WAKEUP_GPIO GPIO_NUM_7  // GPIO pin connected to mechanical push switch
#define LED1 GPIO_NUM_5        // GPIO pin to control the first LED with one pattern
#define LED2 GPIO_NUM_3        // GPIO pin to control the second LED with a different pattern

// Timing definitions
#define LED_PATTERN_DURATION 10000 // Duration for the LED patterns in milliseconds
#define COUNTDOWN_DURATION 10000   // Countdown duration before going back to sleep (10 seconds)

// Global variables
bool pushSwitchActivated = false;
unsigned long lastInterruptTime = 0;

// Function to blink LED1 in a cool pattern
void blinkLED1() {
  for (int i = 0; i < 5; i++) {  // Blink 5 times
    digitalWrite(LED1, HIGH);  // Turn the LED on
    delay(300);                // Wait for 300 milliseconds
    digitalWrite(LED1, LOW);   // Turn the LED off
    delay(300);                // Wait for 300 milliseconds

    digitalWrite(LED1, HIGH);  // Turn the LED on
    delay(100);                // Wait for 100 milliseconds
    digitalWrite(LED1, LOW);   // Turn the LED off
    delay(100);                // Wait for 100 milliseconds
  }
}

// Function to blink LED2 in a different cool pattern
void blinkLED2() {
  delay(1000); // Wait for 1 second before starting the second LED pattern
  for (int i = 0; i < 5; i++) {  // Blink 5 times
    digitalWrite(LED2, HIGH);  // Turn the LED on
    delay(200);                // Wait for 200 milliseconds
    digitalWrite(LED2, LOW);   // Turn the LED off
    delay(200);                // Wait for 200 milliseconds

    digitalWrite(LED2, HIGH);  // Turn the LED on
    delay(50);                 // Wait for 50 milliseconds
    digitalWrite(LED2, LOW);   // Turn the LED off
    delay(50);                 // Wait for 50 milliseconds
  }
}

// Debounce function for GPIO
bool debounceWakeupPin() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastInterruptTime > 250) { // Debounce time
    lastInterruptTime = currentMillis;
    return true; // Accept the new trigger
  }
  return false; // Ignore the trigger
}

void IRAM_ATTR handleInterrupt() {
  // Check if the debounce condition is met
  if (debounceWakeupPin()) {
    // Check the state of GPIO 7 (mechanical switch)
    if (digitalRead(WAKEUP_GPIO) == HIGH) {
      pushSwitchActivated = true;
    }
  }
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  
  pinMode(LED1, OUTPUT);        // GPIO 5 as an output for LED1
  pinMode(LED2, OUTPUT);        // GPIO 3 as an output for LED2
  pinMode(WAKEUP_GPIO, INPUT_PULLUP); // Configure GPIO 7 as input with pull-up resistor

  // Attach interrupt to the wakeup GPIO pin
  attachInterrupt(digitalPinToInterrupt(WAKEUP_GPIO), handleInterrupt, RISING);
}

void loop() {
  // Check if the push switch was activated
  if (pushSwitchActivated) {
    pushSwitchActivated = false; // Reset the flag

    // Print a message to indicate that the ESP32 has woken up
    Serial.println("Woke up from deep sleep!");

    // Start the LED patterns
    blinkLED1(); // LED1 pattern
    blinkLED2(); // LED2 pattern

    // Countdown before going back to sleep
    for (int i = COUNTDOWN_DURATION / 1000; i > 0; i--) {
      Serial.print(i);
      Serial.println(" seconds remaining before going to sleep...");
      delay(1000); // Wait for 1 second
    }

    // Print a message to indicate that the ESP32 is going back to sleep
    Serial.println("Going back to sleep...");
    
    // Enter deep sleep
    esp_deep_sleep_start();
  } else {
    // If no valid interrupt detected, stay in loop
    delay(100); // Short delay to avoid busy-wait loop
  }
}