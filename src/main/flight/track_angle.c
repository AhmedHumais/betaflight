#include "platform.h"

#ifdef USE_TRACK_ANGLE

#include "flight/track_angle.h"

#include <string.h>

#include "common/maths.h"
#include "common/axis.h"

#include "sensors/acceleration.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"

#define TRACK_ANGLE_TIMEOUT_US      (100000)

/* midpoint encoding: 32768 = 0.00 deg */
#define TRACK_ANGLE_MIDPOINT_CDEG   32768.0f
#define TRACK_ANGLE_CDEG_TO_DEG     0.01f

typedef struct trackAngleState_s {
    trackAngleCommand_t command;
    attitudeEulerAngles_t ref_attitude; // in decidegrees
    timeUs_t lastUpdateUs;
    bool hasCommand;
} trackAngleState_t;

static trackAngleState_t trackAngleState;

static float calculateTargetAngleDeg(uint16_t encoded, int axis)
{
    const float centideg = (float)encoded - TRACK_ANGLE_MIDPOINT_CDEG;
    const float errorDeg =  centideg * TRACK_ANGLE_CDEG_TO_DEG;
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
    const float refAngle = (trackAngleState.ref_attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f; // ref_attitude is in decidegrees, convert to degrees before adding error
    return refAngle + errorDeg; // target angle is reference angle + error from tracking
}

static bool trackAngleHasCommand(void)
{
    return trackAngleState.hasCommand;
}

static timeDelta_t trackAngleGetAgeUs(timeUs_t nowUs)
{
    if (!trackAngleState.hasCommand) {
        return TRACK_ANGLE_TIMEOUT_US + 1;
    }

    return cmpTimeUs(nowUs, trackAngleState.lastUpdateUs);
}

static bool trackAngleIsFresh(timeUs_t nowUs)
{
    return trackAngleHasCommand() && (trackAngleGetAgeUs(nowUs) <= TRACK_ANGLE_TIMEOUT_US);
}

static bool trackAngleTrackingHealthy(timeUs_t nowUs)
{
    return trackAngleIsFresh(nowUs)
        && trackAngleState.command.valid
        && !trackAngleState.command.targetLost;
}

void trackAngleInit(void)
{
    trackAngleReset();
}

void trackAngleReset(void)
{
    memset(&trackAngleState, 0, sizeof(trackAngleState));
}

void trackAngleSetCommand(const trackAngleCommand_t *command, timeUs_t nowUs)
{
    if (!command) {
        return;
    }

    trackAngleState.command = *command;
    trackAngleState.ref_attitude = attitude; // capture current attitude as reference
    trackAngleState.lastUpdateUs = nowUs;
    trackAngleState.hasCommand = true;
}

bool trackAngleOverrideActive(timeUs_t nowUs)
{
    return IS_RC_MODE_ACTIVE(BOXTRACKANGLE)
        && FLIGHT_MODE(ANGLE_MODE)
        && !FLIGHT_MODE(HORIZON_MODE | GPS_RESCUE_MODE)
        && !failsafeIsActive()
        && trackAngleTrackingHealthy(nowUs);
}


float trackAngleGetTargetAngleDeg(int axis)
{
    switch (axis) {
    case FD_ROLL:
        return calculateTargetAngleDeg(trackAngleState.command.rollError, axis);
    case FD_PITCH:
        return calculateTargetAngleDeg(trackAngleState.command.pitchError, axis);
    case FD_YAW: // YAW is not used though
        return calculateTargetAngleDeg(trackAngleState.command.yawError, axis);
    default:
        return 0.0f;
    }
}

#endif