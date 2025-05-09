#pragma once

void handleWakeUpInterrupt();
void enterDeepSleep();
bool setupSensors();
bool initializeMPU();
bool significantActivityDetected();
void monitorSensors();
void performFFT();
bool initializeMAX();