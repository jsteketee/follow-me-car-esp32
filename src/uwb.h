#pragma once

#include <Arduino.h>

struct UWBReading {
    float         angleDeg;   // AOA bearing to tag: 0=ahead, +right, -left
    float         distCm;     // Kalman-filtered distance in cm; -1 if no valid reading
    unsigned long timestamp;  // millis() of last valid frame
};

void              uwb_init();
void              uwb_update();
const UWBReading& uwb_get();
