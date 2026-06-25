#pragma once
#include <Arduino.h>

struct CameraData {
    bool  found;
    float posX;       // -1.0 (left) to +1.0 (right)
    float posY;       // -1.0 (bottom) to +1.0 (top)
    unsigned long timestamp;
};

bool camera_init();   // returns true if camera found, false if not present
void camera_update();
const CameraData& camera_get();
