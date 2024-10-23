#include <Arduino.h>
#include "sensorManager.h"
#include "spiffsManager.h"
#include "powerManager.h"
#include "commandHandler.h"
#include "SPIFFS.h"

#ifdef HELTEC_V3_DEVKIT
    #include "/home/anda/Documents/repositories/iot_projects/vibration_detection/boards/heltec_v3/variant.h"
#elif defined(ESP32_S3_DEVKIT)
    #include "/home/anda/Documents/repositories/iot_projects/vibration_detection/boards/esp_devkiS3/variant.h"
#else
    #error "Board not defined. Please define either ESP32_S3_DEVKIT or HELTEC_V3_DEVKIT."
#endif


void setup() {
    Serial.begin(115200);
    setupPower();  // This will handle toggling Vext to power the sensors
    delay(2000);   // Wait for sensor to stabilize before initialization

    // Initialize components
    setupSensors();
    setupSPIFFS();
    setupPower();
    setupCommands();
}

void loop() {
    handleSerialCommands();   // Handle incoming serial commands
    manageBattery();          // Monitor battery and manage power
    monitorSensors();         // Collect and log sensor data
}
