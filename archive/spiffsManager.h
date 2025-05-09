#pragma once
#include <stddef.h>  // Ensure size_t is defined

void setupSPIFFS();
bool logDataToSPIFFS(float *data, size_t length, int phase);
void checkSPIFFSSpace();
void extractDataOverSerial();
void eraseSPIFFSData();
void SPIFFSDebug(const char *errorMessage, int phase);
