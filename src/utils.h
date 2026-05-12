#pragma once
#include <Arduino.h>

// Tracks how often update() is called and exposes the rate in hz.
struct HzTracker {
    uint32_t count = 0;
    uint32_t lastSecond = 0;
    float hz = 0.0f;

    // Call once per event. hz refreshes 10 times per second
    void update() {
        count++;
        uint32_t now = millis();
        if (now - lastSecond >= 100) {
            hz = count * 1000.0f / (now - lastSecond);
            count = 0;
            lastSecond = now;
        }
    }
};
