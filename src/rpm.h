#pragma once

struct RPMData {
    float         hallRawMph;   // instantaneous hall speed from the last accepted pulse — no smoothing
    float         hallSpeedMph; // EMA hall speed with spike rejection; forced to 0 while cogging or pulse-stale
    float         encRawMph;    // signed AS5600 encoder velocity from the last sample (forward = positive) — no smoothing
    float         encSpeedMph;  // EMA of encRawMph; drives cogging exit and offline characterization on the Pi
    float         fusedSpeedMph; // 2-state fused speed [speed, accelBias]: IMU forward accel predicts, encoder (speed-ramped R) + hall correct — the throttle PID's feedback
    float         odometryCm;   // cumulative distance since startup
    unsigned long timestamp;
    bool          cogging;      // true when encoder detects motor oscillation without net forward motion
    int           signChanges;  // direction reversals in the current cogging analysis window
    float         hallHz;       // hall pulse arrival rate
};

void           rpm_init();
bool           rpm_update(float fwdAccelMps2);  // forward linear accel (imu lax, m/s²) drives the 2-state predict; returns true when a new encoder angle was read
const RPMData& rpm_get();
