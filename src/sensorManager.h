#pragma once
#include "MPU6050.h"
#include <variant.h>

extern MPU6050 mpu;
extern volatile bool wakeup_flag;
extern int remainingCycles;
extern float inputBuffer[];

bool setupSensors();
void monitorSensors();
bool initializeMPU();
void readAccelerometerDataForPhase(int phase);
bool checkFIFOOverflow();
void powerCycleMPU(bool on);
void enterDeepSleep();
void handleWakeUpInterrupt();