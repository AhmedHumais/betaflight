#pragma once

#ifdef USE_TRACK_ANGLE

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

typedef struct trackAngleCommand_s {
    uint16_t rollError;   // centidegrees, midpoint encoded
    uint16_t pitchError;  // centidegrees, midpoint encoded
    uint16_t yawError;    // centidegrees, midpoint encoded
    uint8_t valid;
    uint8_t targetLost;
} trackAngleCommand_t;

void trackAngleInit(void);
void trackAngleReset(void);
void trackAngleSetCommand(const trackAngleCommand_t *command, timeUs_t nowUs);
bool trackAngleOverrideActive(timeUs_t nowUs);
float trackAngleGetTargetAngleDeg(int axis);

void trackAngleUpdateCurrentAngles(float cur_angle, float tgt_angle, int axis);
float trackAngleGetThrottleCompensationNormalized(void);
#endif // USE_TRACK_ANGLE