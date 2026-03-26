#include "platform.h"

#ifdef USE_AUTOTRACK

#include "flight/autotrack.h"
#include "flight/failsafe.h"

#include "fc/runtime_config.h"
#include <string.h>

#include "common/maths.h"
#include "common/axis.h"

#define AUTOTRACK_TIMEOUT_US       (100000)   // 100 ms
#define AUTOTRACK_RATE_MIDPOINT            32768.0f
#define AUTOTRACK_RATE_SCALE_CDEGPS_TO_DPS 0.01f
#define AUTOTRACK_MAX_RATE_DPS             3000.0f


typedef struct autotrackState_s {
    autotrackCommand_t command;
    timeUs_t lastUpdateUs;
    bool hasCommand;
} autotrackState_t;

static autotrackState_t autotrackState;
static bool disabledByTargetLoss = false;

static timeDelta_t autotrackGetAgeUs(timeUs_t nowUs)
{
    if (!autotrackState.hasCommand) {
        return AUTOTRACK_TIMEOUT_US + 1;
    }

    return cmpTimeUs(nowUs, autotrackState.lastUpdateUs);
}

static bool autotrackIsFresh(timeUs_t nowUs)
{
    return autotrackState.hasCommand && (autotrackGetAgeUs(nowUs) <= AUTOTRACK_TIMEOUT_US);
}

static bool autotrackTrackingHealthy(timeUs_t nowUs)
{
    return autotrackIsFresh(nowUs) &&
           autotrackState.command.valid &&
           !autotrackState.command.targetLost;
}

static bool autotrackBaseModeIsAcro(void)
{
    return !FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | MAG_MODE | HEADFREE_MODE | PASSTHRU_MODE | FAILSAFE_MODE | GPS_RESCUE_MODE);
}

static float autotrackDecodeRate(uint16_t encodedRate)
{
    const float centiDegPerSecond = (float)encodedRate - AUTOTRACK_RATE_MIDPOINT;
    const float degPerSecond = centiDegPerSecond * AUTOTRACK_RATE_SCALE_CDEGPS_TO_DPS;
    return constrainf(degPerSecond, -AUTOTRACK_MAX_RATE_DPS, AUTOTRACK_MAX_RATE_DPS);
}

void autotrackInit(void)
{
    autotrackReset();
}

void autotrackReset(void)
{
    memset(&autotrackState, 0, sizeof(autotrackState));
    autotrackState.hasCommand = false;
    disabledByTargetLoss = false;
}

bool autotrackDisabledByTargetLoss(void){
    return disabledByTargetLoss;
}

void autotrackSetCommand(const autotrackCommand_t *command, timeUs_t nowUs)
{
    if (!command) {
        return;
    }

    autotrackState.command = *command;
    autotrackState.lastUpdateUs = nowUs;
    autotrackState.hasCommand = true;
}

float autotrackGetAxisRate(const int axis)
{
    switch (axis) {
    case FD_ROLL:
        return autotrackDecodeRate(autotrackState.command.rollRate);   
    case FD_PITCH:
        return autotrackDecodeRate(autotrackState.command.pitchRate);
    case FD_YAW:
    default:
        return autotrackDecodeRate(autotrackState.command.yawRate);
    }
}

bool autotrackOverrideActive(timeUs_t nowUs)
{
    if FLIGHT_MODE(AUTOTRACK_MODE 
        && !FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | GPS_RESCUE_MODE) 
        && !failsafeIsActive()) {
        if (autotrackTrackingHealthy(nowUs) && !disabledByTargetLoss) {
            return true;
        }
        if (autotrackState.command.targetLost) {
            disabledByTargetLoss = true;
            DISABLE_FLIGHT_MODE(AUTOTRACK_MODE);
        }
    }
    return false;
}

bool autotrackPrecheckPassed(timeUs_t nowUs)
{
    return autotrackBaseModeIsAcro()
        && autotrackTrackingHealthy(nowUs) && !disabledByTargetLoss;
}

#endif // USE_AUTOTRACK