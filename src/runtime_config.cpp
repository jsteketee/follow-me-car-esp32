#include "runtime_config.h"

RuntimeConfig rtConfig = {
    .throttleScale          = THROTTLE_SCALE,
    .throttleDeadband       = THROTTLE_Deadband,
    .followDistanceCm       = FOLLOW_DISTANCE_CM,
    .maxDistanceCm          = MAX_DISTANCE_CM,
    .targetSpeedMph         = THROTTLE_PID_TARGET_MPH,
    .kp                     = THROTTLE_PID_KP,
    .ki                     = THROTTLE_PID_KI,
    .kd                     = THROTTLE_PID_KD,
    .throttleFfK            = THROTTLE_FF_K,
    .smoothAlpha            = THROTTLE_SMOOTH_ALPHA,
    .steeringKp             = STEERING_PID_KP,
    .steeringKi             = STEERING_PID_KI,
    .steeringMax            = STEERING_MAX,
    .uwbKalmanQ             = UWB_KALMAN_Q,
    .uwbKalmanR             = UWB_KALMAN_R,
    .uwbOutlierRejectCm     = UWB_OUTLIER_REJECT_CM,
    .fusionQBearingPerSec   = FUSION_KALMAN_Q_BEARING_PER_SEC,
    .fusionRUwb             = FUSION_KALMAN_R_UWB,
    .fusionStaleUncertainty = FUSION_STALE_UNCERTAINTY,
    .fusionInnovEwmaAlpha   = FUSION_INNOV_EWMA_ALPHA,
};
