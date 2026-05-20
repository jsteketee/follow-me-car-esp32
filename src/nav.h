#pragma once

enum class NavState {
    FOLLOW_ME,
    STALE,             // no new UWB data within expected interval
    TEST,
    WAYPOINT,
    STOPPED,
};

struct NavData {
    float         relativeAngle; // degrees: 0=straight ahead, +right, -left
    float         distanceCm;    // straight-line distance from anchor midpoint to tag
    float         headingHold;   // absolute IMU yaw captured when entering a non-NORMAL mode
    float         updateHz;      // rolling average update rate
    NavState      state;
    unsigned long timestamp;
};

void            nav_update();
const NavData&  nav_get();
void            nav_set_mode(NavState mode);
