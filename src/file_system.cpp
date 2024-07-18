#include "file_system.h"
#include "sensor.h"

void initFileSystem() {
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS filesystem");
        return;
    }
}

void logDataToFile(const char* filename, const AccelerometerSample* samples, int sampleCount, float totalWork) {
    File file = SPIFFS.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }

    file.println("Timestamp,X_Acceleration,Y_Acceleration,Z_Acceleration,Work_X,Work_Y,Work_Z,Total_Work");
    for (int i = 0; i < sampleCount; ++i) {
        file.print(samples[i].timestamp);
        file.print(",");
        file.print(samples[i].acceleration[0]);
        file.print(",");
        file.print(samples[i].acceleration[1]);
        file.print(",");
        file.print(samples[i].acceleration[2]);
        file.print(",");
        file.print(samples[i].work[0]);
        file.print(",");
        file.print(samples[i].work[1]);
        file.print(",");
        file.print(samples[i].work[2]);
        file.print(",");
        file.println(totalWork);
    }

    file.close();
}
