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
    float minSpeedMph;
    float maxSpeedMph;
    float kp;
    float ki;
    float kd;
    float throttleDFilterAlpha;
    float smoothAlpha;
    // Steering
    float steeringTrim;
    float steeringKp;
    float steeringKi;
    float steeringMax;
    // UWB filtering
    float uwbOutlierRejectCm;
    // Fused speed KF (accel predict, hall + encoder correct)
    float fusedEncR;
    float fusedHallR;
    float fusedRampStartMph;
    float fusedRampEndMph;
    float fused2QSpeed;
    float fused2QBias;
    // Hall debounce — speed-adaptive ISR quiet-gap window
    float debounceSpeedFactor;
    float debounceMinUs;
    float debounceMaxUs;
};

extern RuntimeConfig rtConfig;
