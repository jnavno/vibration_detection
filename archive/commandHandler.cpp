#include <Arduino.h>
#include "spiffsManager.h"
#include "powerManager.h"
#include "DebugConfiguration.h"

void setupCommands();
void handleSerialCommands();

void setupCommands() {
    LOG_DEBUGLN("Command handler setup complete.");
}

void handleSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "check_memory") {
            checkSPIFFSSpace();
        } else if (command == "erase_spiffs") {
            eraseSPIFFSData();
        } else if (command == "extract_data") {
            extractDataOverSerial();
        } else {
            LOG_DEBUGLN("Unknown command.");
        }
    }
}
