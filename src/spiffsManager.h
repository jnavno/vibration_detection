#pragma once
#include <stddef.h>

void setupSPIFFS();
bool logDataToSPIFFS(float *data, size_t length, int phase);
void checkSPIFFSSpace();
void extractDataOverSerial();
void eraseSPIFFSData();
void sendFileOverSerial(int phase);
void SPIFFSDebug(const char *errorMessage, int phase);