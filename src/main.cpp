#include <Arduino.h>
#include "sensorManager.h"
#include "spiffsManager.h"
#include "powerManager.h"
#include "commandHandler.h"
#include "SPIFFS.h"

#ifdef HELTEC_V3_DEVKIT
    #include "../boards/heltec_v3/variant.h"
#elif defined(ESP32_S3_DEVKIT)
    #include "../boards/esp_devkitS3/variant.h"
#else
    #error "Board not defined. Please define either ESP32_S3_DEVKIT or HELTEC_V3_DEVKIT."
#endif

void setup() {
    Serial.begin(115200);
    Serial.println("Starting setup...");

    setupPower();  // Power up Vext
    delay(3000);   // Allow extra time for MPU6050 to stabilize

    if (!setupSensors()) {
        Serial.println("Sensor setup failed. Restarting...");
        while (1);  // Halt if setup fails, for debugging
    }

    Serial.println("Setting up SPIFFS...");
    setupSPIFFS();

    Serial.println("Setting up commands...");
    setupCommands();
    Serial.println("Setup complete.");
}

void loop() {
    handleSerialCommands();  // Handle SPIFFS related commands
    monitorBattery();         // Monitor battery and manage power
    monitorFor60Sec();       // Collect and log vibration data
}
