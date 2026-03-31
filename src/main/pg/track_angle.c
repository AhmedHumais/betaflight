#include "platform.h"

#ifdef USE_TRACK_ANGLE

#include "pg/track_angle.h"
#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_TEMPLATE(trackAngleConfig_t, trackAngleConfig, PG_TRACK_ANGLE_CONFIG, 0);

PG_RESET_TEMPLATE(trackAngleConfig_t, trackAngleConfig,
    .throttleCompensationEnable = 1,
    .hoverThrottlePermille = 350,
    .maxCompensationPermille = 300,
    .minCosTiltPermille = 300,
    .targetBlendPermille = 400
);

#endif