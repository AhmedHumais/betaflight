#include "platform.h"

#ifdef USE_TRACK_ANGLE

#include "flight/track_angle.h"

#include <string.h>
#include <math.h>
#include "common/maths.h"
#include "common/axis.h"

#include "sensors/acceleration.h"

#include "drivers/time.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "pg/track_angle.h"

#define TRACK_ANGLE_TIMEOUT_US            (50000)    // 50ms timeout for track angle command, after which the command is considered stale and not used for control
#define TRACK_ANGLE_VALID_TIMEOUT_US      (200000)   // 200ms timeout for track angle command to be considered valid, after which the command is considered lost

/* midpoint encoding: 32768 = 0.00 deg */
#define TRACK_ANGLE_MIDPOINT_CDEG   32768.0f
#define TRACK_ANGLE_CDEG_TO_DEG     0.01f

static FAST_DATA_ZERO_INIT float current_thrust = 0;
static FAST_DATA_ZERO_INIT float gravityCompensated_targetAngles[RP_AXIS_COUNT]; // in degrees

static FAST_DATA_ZERO_INIT float cur_roll_angle_deg; // in degrees
static FAST_DATA_ZERO_INIT float cur_pitch_angle_deg; // in degrees
static FAST_DATA_ZERO_INIT float tgt_roll_angle_deg; // in degrees
static FAST_DATA_ZERO_INIT float tgt_pitch_angle_deg; // in degrees

static FAST_DATA_ZERO_INIT float hoverThrottle;
static FAST_DATA_ZERO_INIT float maxComp;
static FAST_DATA_ZERO_INIT float minCosTilt;
static FAST_DATA_ZERO_INIT float targetBlend;

static uint8_t throttleCompensationEnabled;
static uint8_t gravityCompensationEnabled;

typedef struct trackAngleState_s {
    trackAngleCommand_t command;
    // attitudeEulerAngles_t ref_attitude; // in decidegrees
    float ref_roll_deg;
    float ref_pitch_deg;
    timeUs_t lastUpdateUs;
    timeUs_t lastValidCommandUs;
    bool hasCommand;
    bool recycleNeeded;
    bool enabled;
} trackAngleState_t;

static trackAngleState_t trackAngleState;

static float calculateTargetAngleDeg(uint16_t encoded, int axis, const attitudeEulerAngles_t *ref_attitude)
{
    const float centideg = (float)encoded - TRACK_ANGLE_MIDPOINT_CDEG;
    const float errorDeg =  centideg * TRACK_ANGLE_CDEG_TO_DEG;
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
    const float refAngle = (ref_attitude->raw[axis] - angleTrim->raw[axis]) / 10.0f; // ref_attitude is in decidegrees, convert to degrees before adding error
    return refAngle + errorDeg; // target angle is reference angle + error from tracking
}

static timeDelta_t trackAngleGetAgeUs(timeUs_t nowUs, timeUs_t referenceUs)
{
    if (!trackAngleState.hasCommand) {
        return TRACK_ANGLE_TIMEOUT_US + 1;
    }

    return cmpTimeUs(nowUs, referenceUs);
}

static bool trackAngleIsFresh(timeUs_t nowUs)
{
    return (trackAngleGetAgeUs(nowUs, trackAngleState.lastUpdateUs) <= TRACK_ANGLE_TIMEOUT_US) 
    && (trackAngleGetAgeUs(nowUs, trackAngleState.lastValidCommandUs) <= TRACK_ANGLE_VALID_TIMEOUT_US) ;
}

static bool trackAngleTrackingHealthy(timeUs_t nowUs)
{
    if (!trackAngleIsFresh(nowUs) || trackAngleState.command.targetLost){
        return false;
    }
    // return trackAngleState.command.valid;
    return true;
}

static float blendedThrottleCompensation(void)
{

    const float rollCurrentRad = DEGREES_TO_RADIANS(cur_roll_angle_deg);
    const float pitchCurrentRad = DEGREES_TO_RADIANS(cur_pitch_angle_deg);

    const float rollTargetRad = DEGREES_TO_RADIANS(tgt_roll_angle_deg);
    const float pitchTargetRad = DEGREES_TO_RADIANS(tgt_pitch_angle_deg);

    float cosTiltCurrent = cos_approx(rollCurrentRad) * cos_approx(pitchCurrentRad);
    float cosTiltTarget = cos_approx(rollTargetRad) * cos_approx(pitchTargetRad);

    if (cosTiltCurrent < minCosTilt) {
        cosTiltCurrent = minCosTilt;
    }
    if (cosTiltTarget < minCosTilt) {
        cosTiltTarget = minCosTilt;
    }

    const float extraCurrent = hoverThrottle * ((1.0f / cosTiltCurrent) - 1.0f);
    const float extraTarget = hoverThrottle * ((1.0f / cosTiltTarget) - 1.0f);

    float extra = (1.0f - targetBlend) * extraCurrent + targetBlend * extraTarget;
    extra = constrainf(extra, 0.0f, maxComp);

    return extra;
}

/// 

static void trackAngleGetDesiredAccelerationUnitVector(float *a)
{
    const float rollRad = DEGREES_TO_RADIANS(trackAngleState.ref_roll_deg);
    const float pitchRad = DEGREES_TO_RADIANS(trackAngleState.ref_pitch_deg);

    // Direction corresponding to target tilt, expressed in inertial/world frame.
    // This is the same body-z direction formula for yaw = 0:
    // z_b^I = [sin(theta) cos(phi), -sin(phi), cos(theta) cos(phi)]
    const float x = sin_approx(pitchRad) * cos_approx(rollRad);
    const float y = -sin_approx(rollRad);
    const float z = cos_approx(pitchRad) * cos_approx(rollRad);

    if (a) {
        a[0] = x;
        a[1] = y;
        a[2] = z;
    }
}

static bool normalizeVector3f(float *v)
{
    const float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n > 1e-6f) {
        v[0] /= n;
        v[1] /= n;
        v[2] /= n;
        return true;
    } else {
        v[0] = 0.0f;
        v[1] = 0.0f;
        v[2] = 1.0f;
    }
    return false;
}

// static void cross3f(float *a, float *b, float *c)
// {
//     c[0] = a[1] * b[2] - a[2] * b[1];
//     c[1] = a[2] * b[0] - a[0] * b[2];
//     c[2] = a[0] * b[1] - a[1] * b[0];
// }

static void trackAngleGetDesiredThrustDirection(const float desiredAccelG, float *t)
{
    float a[3] = {0.0f, 0.0f, 1.0f};
    trackAngleGetDesiredAccelerationUnitVector(a);

    // Desired acceleration vector in "g units"
    float desAx = desiredAccelG * a[0];
    float desAy = desiredAccelG * a[1];
    float desAz = desiredAccelG * a[2];

    // Add gravity vector (+z world)
    float thrustV[3] = {desAx, desAy, desAz + 1.0f};

    normalizeVector3f(thrustV);

    if (t) {
        t[0] = thrustV[0];
        t[1] = thrustV[1];
        t[2] = thrustV[2];
    }
}

static void desAtoAnglesDeg(const float *desA, float *pitchDeg, float *rollDeg)
{
    const float eps = 1e-6f;

    if (!desA || !pitchDeg || !rollDeg) {
        return;
    }
    float uRaw[3] = {desA[0], desA[1], desA[2]};

    if (!normalizeVector3f(uRaw)) {
        *pitchDeg = 0.0f;
        *rollDeg = 0.0f;
        return;
    }

    const float ux = uRaw[0];
    const float uy = uRaw[1];
    const float rho = sqrtf(ux * ux + uy * uy);

    if (rho < eps) {
        *pitchDeg = 0.0f;
        *rollDeg = 0.0f;
        return;
    }

    const float rhoClamped = constrainf(rho, 0.0f, 1.0f);
    const float alpha = asin_approx(rhoClamped);

    *pitchDeg = RADIANS_TO_DEGREES(alpha * (ux / rho));
    *rollDeg = RADIANS_TO_DEGREES(-alpha * (uy / rho));
}

static void updateGravityCompensatedTargetAngles(const float throttleCommand)
{
    float desA[3];
    const float desiredAccelG = ABS( (throttleCommand - hoverThrottle) /hoverThrottle ); // in "g units", how much more acceleration than hover is needed to achieve the current thrust    
    trackAngleGetDesiredThrustDirection(desiredAccelG, desA);

    desAtoAnglesDeg(desA, &gravityCompensated_targetAngles[AI_PITCH], &gravityCompensated_targetAngles[AI_ROLL]);
}

void trackAngleInit(void)
{
    const trackAngleConfig_t *cfg = trackAngleConfig();

    hoverThrottle = constrainf(cfg->hoverThrottlePermille * 0.001f, 0.0f, 1.0f);
    maxComp = constrainf(cfg->maxCompensationPermille * 0.001f, 0.0f, 1.0f);
    minCosTilt = constrainf(cfg->minCosTiltPermille * 0.001f, 0.05f, 1.0f);
    targetBlend = constrainf(cfg->targetBlendPermille * 0.001f, 0.0f, 1.0f);

    throttleCompensationEnabled = cfg->throttleCompensationEnable;
    gravityCompensationEnabled = cfg->gravityCompensationEnable;

    trackAngleReset();
}

void trackAngleReset(void)
{
    memset(&trackAngleState, 0, sizeof(trackAngleState));
    trackAngleState.recycleNeeded = true;
    trackAngleState.enabled = false;
    trackAngleState.hasCommand = false;
}

void trackAngleSetCommand(const trackAngleCommand_t *command, timeUs_t nowUs)
{
    if (!command) {
        return;
    }

    trackAngleState.command = *command;
    if (command->valid){
        trackAngleState.lastValidCommandUs = nowUs;
        trackAngleState.ref_roll_deg = calculateTargetAngleDeg(command->rollError, FD_ROLL, &attitude);
        trackAngleState.ref_pitch_deg = calculateTargetAngleDeg(command->pitchError, FD_PITCH, &attitude);
        // if (gravityCompensationEnabled) {
        //     updateGravityCompensatedTargetAngles(current_thrust);
        // }
    }
    trackAngleState.lastUpdateUs = nowUs;
    trackAngleState.hasCommand = true;
}

bool trackAngleModeActive(void)
{
    return trackAngleState.enabled;
}

bool trackAngleOverrideActive(timeUs_t nowUs)
{
    if (!IS_RC_MODE_ACTIVE(BOXTRACKANGLE)){
        trackAngleState.recycleNeeded = false;
    }
    else if (trackAngleState.recycleNeeded) {
        return false;
    }
    else if (FLIGHT_MODE(ANGLE_MODE)
        && !FLIGHT_MODE(HORIZON_MODE | GPS_RESCUE_MODE)
        && !failsafeIsActive()){
        if (!trackAngleTrackingHealthy(nowUs) || !ARMING_FLAG(ARMED)) {
            trackAngleReset();
            trackAngleState.recycleNeeded = true;
            trackAngleState.enabled = false;
            return false;
        }
        if (!trackAngleModeActive()) { 
            // if track angle mode just gets enabled, set the flag and initialize the gravity compensated target angles
            trackAngleState.enabled = true;
            updateGravityCompensatedTargetAngles(current_thrust); // initialize gravity compensated target angles when track angle mode just gets enabled
        }
        return true;    
    } else { 
        // if not in angle mode or in failsafe, reset track angle state to be ready for the next time when angle mode is enabled
        if (trackAngleModeActive()) {
            trackAngleReset();
            trackAngleState.recycleNeeded = true;
            trackAngleState.enabled = false;
        }
    }
        return false;

}

float trackAngleGetTargetAngleDeg(int axis)
{
    float targetAngle = 0.0f;
    switch (axis) {
    case FD_ROLL:
        targetAngle = gravityCompensationEnabled == 0? trackAngleState.ref_roll_deg : gravityCompensated_targetAngles[AI_ROLL];
        break;
    case FD_PITCH:
        targetAngle = gravityCompensationEnabled == 0? trackAngleState.ref_pitch_deg : gravityCompensated_targetAngles[FD_PITCH];
        break;
    // case FD_YAW: // YAW is not used though
    //     targetAngle = calculateTargetAngleDeg(trackAngleState.command.yawError, axis);
    default:
        targetAngle = 0.0f;
    }

    return targetAngle;

}

void trackAngleUpdateCurrentAngles(float cur_angle, float tgt_angle, int axis)
{
    switch (axis)
    {
    case FD_ROLL:
        cur_roll_angle_deg = cur_angle;
        tgt_roll_angle_deg = tgt_angle;
        break;
    case FD_PITCH:
        cur_pitch_angle_deg = cur_angle;
        tgt_pitch_angle_deg = tgt_angle;
        break;
    default:
        return;
    }
}

bool trackAngleThrottleAngleCompensationActive(void)
{
    return trackAngleModeActive() && throttleCompensationEnabled;
}

float applyThrottleCompensationForAngle(float throttleInput)
{
    float throttleOutput = throttleInput + blendedThrottleCompensation();
    return throttleOutput;
}

float trackAngleUpdateCurrentThrust(float throttleInput)
{
    // float thrustOutput = throttleInput;
    current_thrust = throttleInput;
    if (trackAngleOverrideActive(micros())){
        if (gravityCompensationEnabled) {
            // thrustOutput = current_thrust;                       // use current thrust which was previously used for setting target angles. This adds one cycle delay in updating commanded thrust.
            updateGravityCompensatedTargetAngles(current_thrust); // for the next pid update cycle
        }
        // if (throttleCompensationEnabled) {
        //     thrustOutput += blendedThrottleCompensation();
        // }
    }

    return throttleInput;
}



// void trackAngleGetAttitudeSetpointDeg(float desiredAccelG, float *rollDeg, float *pitchDeg)
// {
//     float b3[3] = {0.0f, 0.0f, 1.0f};
//     trackAngleGetDesiredThrustDirection(desiredAccelG, b3);

    // // const float yawRad = DEGREES_TO_RADIANS(trackAngleGetTargetYawDeg());
    // const float yawRad = 0.0f;

    // // Desired heading reference in world frame
    // const float cpsi = cos_approx(yawRad);
    // const float spsi = sin_approx(yawRad);

    // // b1_ref = [cos(psi), sin(psi), 0]
    // // Build full desired rotation from b3 (thrust dir) and yaw reference
    // float b2x, b2y, b2z;
    // cross3f(b3x, b3y, b3z, cpsi, spsi, 0.0f, &b2x, &b2y, &b2z);
    // normalizeVector3f(&b2x, &b2y, &b2z);

    // float b1x, b1y, b1z;
    // cross3f(b2x, b2y, b2z, b3x, b3y, b3z, &b1x, &b1y, &b1z);
    // normalizeVector3f(&b1x, &b1y, &b1z);

    // // Rotation matrix body->world, columns are body axes in world frame
    // const float R00 = b1x; const float R01 = b2x; const float R02 = b3x;
    // const float R10 = b1y; const float R11 = b2y; const float R12 = b3y;
    // const float R20 = b1z; const float R21 = b2z; const float R22 = b3z;

    // // ZYX Euler extraction
    // const float pitchRad = asinf(-constrainf(R20, -1.0f, 1.0f));
    // const float rollRad = atan2f(R21, R22);
    // const float yawOutRad = atan2f(R10, R00);

    // if (rollDeg) {
    //     *rollDeg = RADIANS_TO_DEGREES(rollRad);
    // }
    // if (pitchDeg) {
    //     *pitchDeg = RADIANS_TO_DEGREES(pitchRad);
    // }
    // if (yawDeg) {
    //     *yawDeg = RADIANS_TO_DEGREES(yawOutRad);
    // }
// }

// float trackAngleGetGravityCompThrustFeedforwardNormalized(void)
// {
//     const trackAngleConfig_t *cfg = trackAngleConfig();

//     if (!trackAngleGravityCompensationEnabled()) {
//         return 0.0f;
//     }

//     if (!trackAngleOverrideActive(micros())) {
//         return 0.0f;
//     }

//     const float hoverThrottle = constrainf(cfg->hoverThrottlePermille * 0.001f, 0.0f, 1.0f);
//     const float maxComp = constrainf(cfg->maxCompensationPermille * 0.001f, 0.0f, 1.0f);
//     const float minCosTilt = constrainf(cfg->minCosTiltPermille * 0.001f, 0.05f, 1.0f);
//     const float targetBlend = constrainf(cfg->targetBlendPermille * 0.001f, 0.0f, 1.0f);

//     // --- Current attitude tilt projection ---
//     const float rollCurrentRad = DEGREES_TO_RADIANS(attitude.values.roll * 0.1f);
//     const float pitchCurrentRad = DEGREES_TO_RADIANS(attitude.values.pitch * 0.1f);

//     float cosTiltCurrent = cos_approx(rollCurrentRad) * cos_approx(pitchCurrentRad);
//     if (cosTiltCurrent < minCosTilt) {
//         cosTiltCurrent = minCosTilt;
//     }

//     const float extraCurrent = hoverThrottle * ((1.0f / cosTiltCurrent) - 1.0f);

//     // --- Target attitude tilt projection ---
//     float ax = 0.0f;
//     float ay = 0.0f;
//     float az = 1.0f;
//     trackAngleGetDesiredAccelerationUnitVector(&ax, &ay, &az);

//     float cosTiltTarget = az;
//     if (cosTiltTarget < minCosTilt) {
//         cosTiltTarget = minCosTilt;
//     }

//     const float extraTarget = hoverThrottle * ((1.0f / cosTiltTarget) - 1.0f);

//     // --- Blend current and target compensation ---
//     float extra = (1.0f - targetBlend) * extraCurrent + targetBlend * extraTarget;

//     // --- Final clamp ---
//     extra = constrainf(extra, 0.0f, maxComp);

//     return extra;
// }

// float trackAngleGetGravityCompThrustFeedforwardNormalized(float desiredAccelG)
// {
//     const trackAngleConfig_t *cfg = trackAngleConfig();

//     if (!trackAngleGravityCompensationEnabled()) {
//         return 0.0f;
//     }

//     if (!trackAngleOverrideActive(micros())) {
//         return 0.0f;
//     }

//     float ax = 0.0f, ay = 0.0f, az = 1.0f;
//     trackAngleGetDesiredAccelerationUnitVector(&ax, &ay, &az);

//     // Total thrust vector magnitude in g units
//     const float thrustX = desiredAccelG * ax;
//     const float thrustY = desiredAccelG * ay;
//     const float thrustZ = desiredAccelG * az + 1.0f;

//     const float thrustMag = sqrtf(thrustX * thrustX + thrustY * thrustY + thrustZ * thrustZ);

//     const float hoverThrottle = constrainf(cfg->hoverThrottlePermille * 0.001f, 0.0f, 1.0f);
//     const float maxComp = constrainf(cfg->maxCompensationPermille * 0.001f, 0.0f, 1.0f);

//     float extra = hoverThrottle * (thrustMag - 1.0f);
//     extra = constrainf(extra, 0.0f, maxComp);

//     return extra;
// }

#endif // USE_TRACK_ANGLE