#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>
#include "config.h"

extern volatile bool triggered;

void printWakeupReason();
void handleInterrupt();
void toggleAccelPower(bool state);

#endif // UTILITIES_H
