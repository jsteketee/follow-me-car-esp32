#pragma once
#include <Arduino.h>

// 1-D Kalman filter for scalar measurements.
// Q = process noise variance (how much the true value can change per step).
// R = measurement noise variance.
struct KalmanFilter {
    float x = 0.0f;
    float p = 1000.0f; // large initial uncertainty so first measurement is trusted fully
    bool  initialized = false;

    float update(float z, float q, float r) {
        if (!initialized) { x = z; initialized = true; return x; }
        p += q;
        float k = p / (p + r);
        x += k * (z - x);
        p *= (1.0f - k);
        return x;
    }
};

// Tracks avg and max execution time per step. Call begin() before and end() after each call, reset() after reporting.
struct PerfTracker {
    uint32_t _start = 0;
    uint32_t maxUs  = 0;
    uint32_t sumUs  = 0;
    uint32_t count  = 0;
    void begin() { _start = micros(); }
    void end()   { uint32_t us = micros() - _start; if (us > maxUs) maxUs = us; sumUs += us; count++; }
    uint32_t avg() const { return count ? sumUs / count : 0; }
    void reset() { maxUs = 0; sumUs = 0; count = 0; }
};

// Tracks how often update() is called and exposes the rate in hz.
struct HzTracker {
    uint32_t count = 0;
    uint32_t lastSecond = 0;
    float hz = 0.0f;

    // Call once per event. hz refreshes 10 times per second
    void update(bool valid = true) {
        if (valid) count++;
        uint32_t now = millis();
        if (now - lastSecond >= 100) {
            hz = count * 1000.0f / (now - lastSecond);
            count = 0;
            lastSecond = now;
        }
    }
};
