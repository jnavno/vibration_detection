#ifndef WORK_COMPUTATION_H
#define WORK_COMPUTATION_H

#include <math.h>
#include "AccelBuffer.h"

float computeWork(AccelData* dataSet, int dataSize, float weight) {
    float totalWorkX = 0;
    float totalWorkY = 0;
    float totalWorkZ = 0;
    
    for (int i = 0; i < dataSize; i++) {
        totalWorkX += weight * fabs(dataSet[i].x);
        totalWorkY += weight * fabs(dataSet[i].y);
        totalWorkZ += weight * fabs(dataSet[i].z);
    }

    return totalWorkX + totalWorkY + totalWorkZ;
}

#endif // WORK_COMPUTATION_H
