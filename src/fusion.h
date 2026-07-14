#pragma once
#include <Arduino.h>

struct Pose {
    float         fusedAngle;      // Kalman-filtered degrees: 0=ahead, +right, -left
    float         uwbAngle;        // latest raw UWB AOA angle (pre-fusion); NAN until first fix
    float         distanceCm;      // Kalman-filtered distance (cm); -1 if no valid UWB reading
    float         uwbDistCm;       // latest raw UWB distance (cm); -1 if no valid reading
    float         fusedSpeedMph;   // RPM speed (mph)
    float         fusedOdometryCm; // cumulative wheel odometry (cm)
    unsigned long timestamp;       // millis() of last valid UWB update
    float         uncertainty;     // bearing Kalman variance (deg²) — low = confident, high = stale/uncertain
};

void             fusion_init();
bool             fusion_update();  // true when UWB provided new data this call
const Pose& fusion_get();
