#ifndef ACCEL_BUFFER_H
#define ACCEL_BUFFER_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define INSPECTION_DURATION_SECONDS 11  // Duration to buffer data
#define SAMPLE_RATE_HZ 15  // Sample rate for accelerometer
#define FIFO_SIZE_BYTES 1024  // Maximum FIFO size

Adafruit_MPU6050 mpu;

struct AccelData {
    float x;
    float y;
    float z;
};

// Initialize the accelerometer and FIFO
bool initializeAccelFIFO() {
    if (!mpu.begin()) return false;

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setRate(1000 / SAMPLE_RATE_HZ);
    mpu.setFIFOMode(true);
    mpu.setAccelFIFOEnabled(true);
    mpu.setGyroFIFOEnabled(false);
    mpu.resetFIFO();
    return true;
}

// Collect accelerometer data and return the dataset
bool collectAccelData(AccelData* dataSet, int& dataSize) {
    uint8_t fifoBuffer[FIFO_SIZE_BYTES];
    int packetSize = 6;
    mpu.resetFIFO();
    delay(INSPECTION_DURATION_SECONDS * 1000);

    uint16_t fifoCount = mpu.getFIFOCount();
    if (fifoCount == 0) return false;

    int idx = 0;
    while (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        dataSet[idx].x = ((int16_t)(fifoBuffer[0] << 8 | fifoBuffer[1])) * mpu.getAccelerometerRange() / 32768.0;
        dataSet[idx].y = ((int16_t)(fifoBuffer[2] << 8 | fifoBuffer[3])) * mpu.getAccelerometerRange() / 32768.0;
        dataSet[idx].z = ((int16_t)(fifoBuffer[4] << 8 | fifoBuffer[5])) * mpu.getAccelerometerRange() / 32768.0;
        idx++;
    }
    dataSize = idx;
    return true;
}

#endif // ACCEL_BUFFER_H
