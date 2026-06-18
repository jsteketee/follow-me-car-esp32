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

// 2-state Kalman filter: state = [angle (deg), gyroBias (deg/s)]
// predict() driven by IMU gyro rate; correct() called per sensor measurement.
struct AngleKalman {
    float angle    = 0.0f;
    float gyroBias = 0.0f;
    float P[2][2]  = {{1.0f, 0.0f}, {0.0f, 1.0f}};

    void predict(float gyroRateDegps, float dt, float qAngle, float qBias) {
        angle += (gyroRateDegps - gyroBias) * dt;
        float dt2 = dt * dt;
        float p00 = P[0][0] - dt*P[1][0] - dt*P[0][1] + dt2*P[1][1] + qAngle;
        float p01 = P[0][1] - dt*P[1][1];
        float p10 = P[1][0] - dt*P[1][1];
        float p11 = P[1][1] + qBias;
        P[0][0]=p00; P[0][1]=p01; P[1][0]=p10; P[1][1]=p11;
    }

    void correct(float measurement, float r) {
        float y  = measurement - angle;
        float S  = P[0][0] + r;
        float K0 = P[0][0] / S;
        float K1 = P[1][0] / S;
        angle    += K0 * y;
        gyroBias += K1 * y;
        float p00 = P[0][0] - K0*P[0][0];
        float p01 = P[0][1] - K0*P[0][1];
        float p10 = P[1][0] - K1*P[0][0];
        float p11 = P[1][1] - K1*P[0][1];
        P[0][0]=p00; P[0][1]=p01; P[1][0]=p10; P[1][1]=p11;
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

// Rate gate. Returns true and writes elapsed dt (seconds) when the interval has elapsed.
// Usage: static RateGate gate{20}; float dt; if (!gate.tick(dt)) return;
struct RateGate {
    uint32_t intervalMs;
    uint32_t _lastMs    = 0;

    bool tick(float& dt) {
        uint32_t now = millis();
        if (now - _lastMs < intervalMs) return false;
        dt = (now - _lastMs) / 1000.0f;
        _lastMs = now;
        return true;
    }
};

// PID controller. Derivative is on measurement (not error) to avoid setpoint-change spikes.
// Call reset() when re-entering active control to clear stale integrator state.
struct PidController {
    float kp = 0, ki = 0, kd = 0;
    float integral   = 0;
    float lastMeasure = 0;
    bool  initialized = false;

    float update(float setpoint, float measure, float dt) {
        if (!initialized) { lastMeasure = measure; initialized = true; }
        if (dt > 0.1f) dt = 0.1f;  // clamp first-tick blowup when RateGate _lastMs starts at 0
        float error = setpoint - measure;
        integral += error * dt;
        integral = constrain(integral, -1.0f, 1.0f);
        float derivative = -(measure - lastMeasure) / dt;
        lastMeasure = measure;
        return kp * error + ki * integral + kd * derivative;
    }

    void reset() { integral = 0; lastMeasure = 0; initialized = false; }
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
