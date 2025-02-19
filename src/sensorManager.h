#pragma once

void handleWakeUpInterrupt();
void enterDeepSleep();
bool setupSensors();
bool initializeMPU();
bool significantActivityDetected();
void monitorFor60Sec();
void performFFT();
