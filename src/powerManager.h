#pragma once

void setupPower();
void monitorBattery();
void toggleSensorPower(bool state);
void quickBlinkAndHalt();
void SensorInitOKBlink();
void setVoltageAlertThresholds(float minVoltage, float maxVoltage);
void printResults();