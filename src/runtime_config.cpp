#include "runtime_config.h"

RuntimeConfig rtConfig = {
    .throttleScale    = THROTTLE_SCALE,
    .throttleDeadband = THROTTLE_Deadband,
    .followDistanceCm = FOLLOW_DISTANCE_CM,
    .maxDistanceCm    = MAX_DISTANCE_CM,
    .targetSpeedMph   = THROTTLE_PID_TARGET_MPH,
    .kp               = THROTTLE_PID_KP,
    .ki               = THROTTLE_PID_KI,
    .kd               = THROTTLE_PID_KD,
    .throttleFfK      = THROTTLE_FF_K,
    .smoothAlpha      = THROTTLE_SMOOTH_ALPHA,
};
