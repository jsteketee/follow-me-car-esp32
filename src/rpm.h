#pragma once

struct RPMData {
    float         hallRawMph;   // instantaneous hall speed from the last debounced pulse — no smoothing
    float         hallSpeedMph; // raw hall speed; forced to 0 while cogging or pulse-stale
    float         encRawMph;    // signed AS5600 encoder velocity from the last sample (forward = positive) — no smoothing
    float         encSpeedMph;  // EMA of encRawMph; drives cogging exit and offline characterization on the Pi
    float         fusedSpeedMph; // 2-state fused speed [speed, accelBias]: IMU forward accel predicts, encoder (speed-ramped R) + hall correct — the throttle PID's feedback
    float         fusedNoImuMph; // same 2-state filter with encoder + hall corrections but NO IMU predict — bench diagnostic (on a stand the IMU accel is spurious)
    float         odometryCm;   // cumulative distance since startup
    unsigned long timestamp;
    bool          cogging;      // true when encoder detects motor oscillation without net forward motion
    bool          encoderHealthy; // AS5600 reachable over I2C and magnet present; when false the phantom-odom gate is off (odom runs unguarded)
    int           signChanges;  // direction reversals in the current cogging analysis window
    float         hallHz;       // hall pulse arrival rate
    unsigned long hallPulses;   // cumulative hall pulses that passed the accel gate — dashboard tick markers
    long          encCounts;    // cumulative signed AS5600 counts (4096/rev) since boot — calibration tick counter
    int           encAngle;     // latest AS5600 angle reading (0-4095) — feeds the dashboard angle graph
};

void           rpm_init();
bool           rpm_update(float fwdAccelMps2);  // forward linear accel (imu lax, m/s²) drives the 2-state predict; returns true when a new encoder angle was read
const RPMData& rpm_get();
