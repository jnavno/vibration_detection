#include "spiffsManager.h"
#include "SPIFFS.h"
#include <variant.h>


void setupSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
    }
}

bool logDataToSPIFFS(float *data, size_t length, int phase) {
    char filename[32];
    snprintf(filename, sizeof(filename), "/data_phase_%d.csv", phase);
    File file = SPIFFS.open(filename, FILE_WRITE);
    if (!file) return false;

    file.printf("Time(s), X-axis (g)\n");
    for (size_t i = 0; i < length; i++) {
        file.printf("%f,%f\n", ((float)i / SAMPLE_RATE), data[i]);
    }
    file.close();
    return true;
}

void checkSPIFFSSpace() {
    size_t totalBytes = SPIFFS.totalBytes();
    size_t usedBytes = SPIFFS.usedBytes();
    float usage = (float)usedBytes / totalBytes * 100;
    Serial.printf("SPIFFS Usage: %.2f%%\n", usage);
}

void extractDataOverSerial() {
    for (int phase = 1; phase <= CYCLES_FOR_5_MIN; phase++) {
        char filename[32];
        snprintf(filename, sizeof(filename), "/data_phase_%d.csv", phase);
        File file = SPIFFS.open(filename, FILE_READ);
        if (!file) {
            Serial.println("Failed to open file for reading");
            continue;
        }
        while (file.available()) {
            Serial.write(file.read());
        }
        file.close();
    }
}

void eraseSPIFFSData() {
    SPIFFS.format();
    Serial.println("SPIFFS erased.");
}

// This logs debug information to SPIFFS
void SPIFFSDebug(const char *errorMessage, int phase)
{
    File debugFile = SPIFFS.open("/debug_log.txt", FILE_APPEND);
    if (!debugFile)
    {
        Serial.println("Failed to open debug log file.");
        return;
    }
    debugFile.printf("Error: %s during phase %d\n", errorMessage, phase);
    debugFile.close();
}