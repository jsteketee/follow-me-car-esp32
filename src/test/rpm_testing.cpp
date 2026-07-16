// Archived speed-filter experiments from the 2026-07-16 fusion session — kept for
// reference, excluded from all builds (src/test/ is filtered out; no env compiles this).
// Both filters were superseded by the 2-state [speed, accelBias] fused filter
// (_fused2Kf in rpm.cpp), which became the throttle PID's feedback.
//
// 1. Experimental encoder KF (_encKf): 1-D KalmanFilter over the raw AS5600 velocity,
//    graphed beside the EMA for tuning. Dashboard-tuned values it produced (kept live
//    as the fused filter's encoder R): Q = 1.26e-4, R = 7.94e-2.
// 2. 1-D fused speed (_fusedKf): hall + encoder through one 1-D KF with the
//    speed-ramped encoder R (fused_enc_r) — same measurement model the 2-state filter
//    still uses. Worked well in cruise but lagged hard decels above the ramp: with no
//    accel prediction, the hall-only regime only learns of a decel one stretched pulse
//    interval at a time (see benchmarks.md-era screenshots, 2026-07-16).

#if 0  // reference only — depends on rpm.cpp statics that no longer exist

// --- declarations (rpm.cpp top) ---
static KalmanFilter _encKf;    // experimental 1-D KF on the raw encoder velocity
static KalmanFilter _fusedKf;  // hall + encoder fused speed, 1-D

// RPMData fields (rpm.h):
//   float kfEncSpeedMph;  // raw encoder velocity through the 1-D KF
//   float fusedSpeedMph;  // hall + encoder 1-D fusion

// config.h defaults:
//   #define RPM_ENC_KF_Q    0.000126f
//   #define RPM_ENC_KF_R    0.0794f
//   #define RPM_FUSED_KF_Q  0.000126f  // rtConfig.fusedKfQ, dashboard log slider

// --- encoder hook (update_encoder, after rawVelMph) ---
_rpmData.kfEncSpeedMph = _encKf.update(rawVelMph, rtConfig.encKfQ, rtConfig.encKfR);
float fusedEncR = fused_enc_r(_fusedKf.x);
if (!isnan(fusedEncR))
    _rpmData.fusedSpeedMph = _fusedKf.update(rawVelMph, rtConfig.fusedKfQ, fusedEncR);

// --- hall hook (update_hall, per accepted pulse) ---
_rpmData.fusedSpeedMph = _fusedKf.update(rawSpeed, rtConfig.fusedKfQ, rtConfig.fusedHallR);

// --- stale-hall zero injection (update_hall, stale/cogging path) ---
if (fabsf(_fusedKf.x) > rtConfig.fusedRampStartMph)
    _rpmData.fusedSpeedMph = _fusedKf.update(0.0f, rtConfig.fusedKfQ, rtConfig.fusedHallR);

#endif
