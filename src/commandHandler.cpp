#include <Arduino.h>
#include "commandHandler.h"
#include "spiffsManager.h"
#include "powerManager.h"

void setupCommands() {
    Serial.println("Command handler setup complete.");
}

void handleSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();  // Remove any extra newline or spaces

        if (command == "check_memory") {
            checkSPIFFSSpace();
        } else if (command == "erase_spiffs") {
            eraseSPIFFSData();
        } else if (command == "extract_data") {
            extractDataOverSerial();
        } else {
            Serial.println("Unknown command.");
        }
    }
}
