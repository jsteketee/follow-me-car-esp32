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
    float steeringTrim;
    float steeringKp;
    float steeringKi;
    float steeringMax;
    // UWB filtering
    float uwbKalmanQ;
    float uwbKalmanR;
    float uwbOutlierRejectCm;
    // Fusion
    float sensorTimeoutSec;
    float fusionRUwb;
    float fusionRCamera;
    float fusionStaleUncertainty;
    float fusionInnovMeanAlpha;
    float fusionInnovEwmaAlpha;
};

extern RuntimeConfig rtConfig;
