#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H

#include "SPIFFS.h"
#include "sensor.h"

void initFileSystem();
void logDataToFile(const char* filename, const AccelerometerSample* samples, int sampleCount, float totalWork);

#endif // FILE_SYSTEM_H
