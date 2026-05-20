#pragma once
#include "config.h"

// Mutable runtime copies of config values. Modified by the dashboard; reset to
// config.h defaults on reboot.
struct RuntimeConfig {
    float throttleScale;
    float throttleDeadband;
    float followDistanceCm;
    float maxDistanceCm;
    float targetSpeedMph;
    float kp;
    float ki;
    float kd;
    float throttleFfK;    // throttle feed-forward gain (targetSpeedMph → throttle)
    float smoothAlpha;    // exponential smoothing on throttle output (0=frozen, 1=no smoothing)
};

extern RuntimeConfig rtConfig;
