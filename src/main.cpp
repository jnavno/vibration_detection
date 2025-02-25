#include <Arduino.h>
#include "sensorManager.h"
#include "spiffsManager.h"
#include "powerManager.h"
#include "commandHandler.h"
#include "SPIFFS.h"
#include "DebugConfiguration.h"

#ifdef HELTEC_V3_DEVKIT
    #include "../boards/heltec_v3/variant.h"
#elif defined(ESP32_S3_DEVKIT)
    #include "../boards/esp_devkitS3/variant.h"
#else
    #error "Board not defined. Please define either ESP32_S3_DEVKIT or HELTEC_V3_DEVKIT."
#endif

void setup() {
    Serial.begin(SERIAL_BAUD);
    debug_println("Starting setup...");

    setupPower();  // Power up Vext
    delay(3000);   // Allow extra time for MPU6050 to stabilize

    if (!setupSensors()) {
        debug_println("Sensor setup failed. Restarting...");
        while (1);  // Halt if setup fails, for debugging
    }
    debug_println("Setting up SPIFFS...");
    setupSPIFFS();
    debug_println("Setting up commands...");
    setupCommands();
    debug_println("Setup complete.");
}

void loop() {
    handleSerialCommands();  // Handle SPIFFS related commands
    monitorSensors();   // Monitors battery, accelerometer, FFT, and dynamic sampling.
}
