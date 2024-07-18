#ifndef SENSOR_H
#define SENSOR_H

#include <Adafruit_MPU6050.h>
#include "config.h"

typedef struct {
    float acceleration[3];
    float work[3];
    unsigned long timestamp;
} AccelerometerSample;

class SensorManager {
public:
    SensorManager();
    bool inspectTreeShaking();

private:
    float weight;
    float totalWorkX;
    float totalWorkY;
    float totalWorkZ;
    float totalWork;
    Adafruit_MPU6050 mpu;
};

#endif // SENSOR_H
