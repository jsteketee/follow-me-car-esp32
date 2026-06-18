#pragma once
#include <Arduino.h>

struct Pose {
    float         angle;        // Kalman-filtered degrees: 0=ahead, +right, -left
    float         distanceCm;   // from UWB distFast; -1 if no valid UWB reading
    unsigned long timestamp;    // millis() of last valid UWB or camera update
    float         uncertainty;  // bearing Kalman variance (deg²) — low = confident, high = stale/uncertain
};

void             fusion_init();
void             fusion_update();
const Pose& fusion_get();
