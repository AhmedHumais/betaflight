#pragma once

#ifdef USE_AUTOTRACK

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

typedef struct autotrackCommand_s {
    uint16_t rollRate;
    uint16_t pitchRate;
    uint16_t yawRate;
    uint8_t valid;
    uint8_t targetLost;
} autotrackCommand_t;

void autotrackInit(void);
void autotrackReset(void);
bool autotrackDisabledByTargetLoss(void);
// bool autotrackIsValid(timeUs_t nowUs);
bool autotrackPrecheckPassed(timeUs_t nowUs);
// void autotrackInvalidate(void);
bool autotrackOverrideActive(timeUs_t nowUs);
void autotrackSetCommand(const autotrackCommand_t *command, timeUs_t nowUs);
float autotrackGetAxisRate(const int axis);

#endif // USE_AUTOTRACK