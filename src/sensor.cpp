#include "sensor.h"
#include "file_system.h"

SensorManager::SensorManager() : weight(DEVICE_WEIGHT), totalWorkX(0.0), totalWorkY(0.0), totalWorkZ(0.0), totalWork(0.0) {
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1); // Halt execution if MPU6050 is not found
    }
}

bool SensorManager::inspectTreeShaking() {
    unsigned long startTime = millis();
    unsigned long elapsedTime = 0;
    int sample_count = 0;
    AccelerometerSample samples[MAX_SAMPLES_PER_CYCLE];

 
    Serial.println("Checking for tree shaking...");

    while (elapsedTime < INSPECTION_DURATION_SECONDS * 1000 && sample_count < MAX_SAMPLES_PER_CYCLE) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        float workX = weight * fabs(a.acceleration.x);
        float workY = weight * fabs(a.acceleration.y);
        float workZ = weight * fabs(a.acceleration.z);

        totalWorkX += workX;
        totalWorkY += workY;
        totalWorkZ += workZ;
        totalWork += workX + workY + workZ;

        samples[sample_count].acceleration[0] = a.acceleration.x;
        samples[sample_count].acceleration[1] = a.acceleration.y;
        samples[sample_count].acceleration[2] = a.acceleration.z;
        samples[sample_count].work[0] = workX;
        samples[sample_count].work[1] = workY;
        samples[sample_count].work[2] = workZ;
        samples[sample_count].timestamp = millis();

        sample_count++;
        elapsedTime = millis() - startTime;
        delay(100);
    }

    logDataToFile(CSV_FILENAME, samples, sample_count, totalWork);

    Serial.println("Total work done on X axis: " + String(totalWorkX));
    Serial.println("Total work done on Y axis: " + String(totalWorkY));
    Serial.println("Total work done on Z axis: " + String(totalWorkZ));
    Serial.println("Total work done on all axes: " + String(totalWork));

    if (sample_count < MAX_SAMPLES_PER_CYCLE) {
        Serial.println("Not enough samples collected. Exiting.");
        return false;
    }

    return true;
}
