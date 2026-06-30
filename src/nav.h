#pragma once
#include "fusion.h"

enum class NavMode {
    FOLLOW_ME,
    TEST,
    THROTTLE_TEST,
    STOPPED,
};

struct NavData {
    float    headingHold;   // absolute IMU yaw captured when entering a target heading mode
    float    updateHz;      // rolling average update rate
    NavMode  mode;
    bool     sensorsValid;  // false when fusion uncertainty exceeds FUSION_STALE_UNCERTAINTY
};

void            nav_update();
const NavData&  nav_get();
void            nav_set_mode(NavMode mode);
