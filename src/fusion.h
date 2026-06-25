#pragma once
#include <Arduino.h>

struct Pose {
    float         fusedAngle;      // Kalman-filtered degrees: 0=ahead, +right, -left
    float         uwbAngle;        // latest UWB trilateration angle (post-UWB-Kalman, pre-fusion); NAN until first fix
    float         camAngle;        // latest raw camera angle; NAN until first fix
    float         distanceCm;      // from UWB distFast; -1 if no valid UWB reading
    float         fusedSpeedMph;   // RPM speed zeroed during cogging
    float         fusedOdometryCm; // cumulative distance, not incremented during cogging
    unsigned long timestamp;       // millis() of last valid UWB or camera update
    float         uncertainty;     // bearing Kalman variance (deg²) — low = confident, high = stale/uncertain
};

void             fusion_init();
void             fusion_update();
const Pose& fusion_get();
