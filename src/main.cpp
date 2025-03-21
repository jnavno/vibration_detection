#include <Wire.h>
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
    INIT_DEBUG_SERIAL();
    LOG_DEBUGLN("Starting setup...");

    setupPower();  // Power up Vext
    toggleSensorPower(true);
    delay(3000);   // Allow extra time for MPU6050 to stabilize
    // Initialize I2C early with defined SDA and SCL pins and set the clock.
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    delay(3000);
    if (!setupSensors()) {
        LOG_DEBUGLN("Sensor setup failed. Restarting...");
        while (1);  // Halt if setup fails, for debugging
    }
    LOG_DEBUGLN("Setting up SPIFFS...");
    setupSPIFFS();
    LOG_DEBUGLN("Setting up commands...");
    setupCommands();
    LOG_DEBUGLN("Setup complete.");
}

void loop() {
    handleSerialCommands();  // Handle SPIFFS related commands
    monitorSensors();   // Monitors battery, accelerometer, FFT, and dynamic sampling.
}
