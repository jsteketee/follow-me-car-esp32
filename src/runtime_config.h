#pragma once
#include "config.h"

// Mutable runtime copies of config values. Modified by the dashboard; reset to
// config.h defaults on reboot.
struct RuntimeConfig {
    // Throttle
    float throttleScale;
    float throttleDeadband;
    float followDistanceCm;
    float maxDistanceCm;
    float targetSpeedMph;
    float kp;
    float ki;
    float kd;
    float throttleFfK;
    float smoothAlpha;
    // Steering
    float steeringKp;
    float steeringKi;
    float steeringMax;
    // UWB filtering
    float uwbKalmanQ;
    float uwbKalmanR;
    float uwbOutlierRejectCm;
    // Fusion
    float fusionQBearingPerSec;
    float fusionRUwb;
    float fusionStaleUncertainty;
    float fusionInnovEwmaAlpha;
};

extern RuntimeConfig rtConfig;
