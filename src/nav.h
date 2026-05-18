#pragma once
#include "uwb.h"
#include "imu.h"

enum class NavState {
    VALID,
    STALE,             // no new UWB data within expected interval
};

struct NavData {
    float         relativeAngle; // degrees: 0=straight ahead, +right, -left
    float         distanceCm;    // straight-line distance from anchor midpoint to tag
    float         updateHz;      // rolling average update rate
    NavState      state;
    unsigned long timestamp;
};

void            nav_update(const UWBReading& uwb, const ImuData& imu);
const NavData&  nav_get();
