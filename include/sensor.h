#ifndef SENSOR_H
#define SENSOR_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define MAX_SAMPLES_PER_CYCLE 100

struct AccelerometerSample {
    float acceleration[3]; // x, y, z
    float work[3];         // work_x, work_y, work_z
    unsigned long timestamp;
};

class SensorManager {
public:
    SensorManager();
    bool inspectTreeShaking();

private:
    Adafruit_MPU6050 mpu;
    float weight;
    float totalWorkX;
    float totalWorkY;
    float totalWorkZ;
    float totalWork;
};

#endif // SENSOR_H
