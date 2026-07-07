#pragma once
#include <Arduino.h>

struct CameraData {
    bool  found;
    float posX;       // -1.0 (left) to +1.0 (right)
    float posY;       // -1.0 (bottom) to +1.0 (top)
    unsigned long timestamp;
};

bool camera_init();     // always returns true; camera_update() retries until device appears
bool camera_update();   // true when a complete sensor read completed (blob or not); false if rate-gated or still retrying
const CameraData& camera_get();
