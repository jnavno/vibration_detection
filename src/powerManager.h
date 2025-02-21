#pragma once

void setupPower();
void monitorBattery();
void toggleSensorPower(bool state);
void quickBlinkAndHalt();
void quickStatusOnBlink();
void setVoltageAlertThresholds(float minVoltage, float maxVoltage);
void printResults();