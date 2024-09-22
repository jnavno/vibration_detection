#ifndef THRESHOLD_EVALUATION_H
#define THRESHOLD_EVALUATION_H

#include "WorkComputation.h"

#define FIRST_THRESHOLD 100.0
#define SECOND_THRESHOLD 200.0
#define THIRD_THRESHOLD 300.0

enum WorkStage {
    STAGE_1,
    STAGE_2,
    STAGE_3
};

// Check if work exceeds the thresholds
bool checkThreshold(float totalWork, WorkStage stage) {
    switch(stage) {
        case STAGE_1:
            return totalWork > FIRST_THRESHOLD;
        case STAGE_2:
            return totalWork > SECOND_THRESHOLD;
        case STAGE_3:
            return totalWork > THIRD_THRESHOLD;
        default:
            return false;
    }
}

#endif // THRESHOLD_EVALUATION_H
