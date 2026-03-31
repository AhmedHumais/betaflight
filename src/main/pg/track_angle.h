#pragma once

#ifdef USE_TRACK_ANGLE

#include <stdint.h>

#include "pg/pg.h"

typedef struct trackAngleConfig_s {
    uint8_t throttleCompensationEnable;
    uint16_t hoverThrottlePermille;          // 350 = 0.350 normalized hover throttle
    uint16_t maxCompensationPermille;        // 300 = max +0.300 normalized throttle
    uint16_t minCosTiltPermille;             // 300 = minimum cos tilt = 0.300
    uint16_t targetBlendPermille;           // 0..1000
} trackAngleConfig_t;

PG_DECLARE(trackAngleConfig_t, trackAngleConfig);

#endif