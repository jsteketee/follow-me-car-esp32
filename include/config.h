#pragma once

// Ordered to follow the data flow: pins → comms → sensors → display →
// control → actuators.

// =============================================================================
// Pin Assignments
// =============================================================================

// I2C — sensor bus (Wire: BNO085 + AS5600)
#define PIN_SDA               8
#define PIN_SCL               9

// I2C — OLED bus (Wire1: dedicated controller so the ~25ms frame push never
// contends with sensor traffic)
#define PIN_OLED_SDA         17
#define PIN_OLED_SCL         18

// DW3000 UWB UART
#define PIN_DW3000_RX         4
#define PIN_DW3000_TX         5

// PWM outputs
#define PIN_ESC               1
#define PIN_SERVO             2
#define PIN_SERVO_UWB         6   // pan servo aiming the UWB anchor (AOA degrades past ~±60°)

// Misc
#define PIN_BUZZER           12
#define PIN_LED              13
#define PIN_RPM              10

// =============================================================================
// WiFi
// =============================================================================
#include "secrets.h"
#define WIFI_AP_SSID        "Follow Me"
#define WIFI_AP_PASS        ""
#define WIFI_HOSTNAME       "followme"   // mDNS/DHCP hostname — no spaces (http://followme.local/)
#define TELNET_PORT         23
#define WIFI_STA_TIMEOUT_MS 7000   // per-network connect timeout before falling back to the next option

// =============================================================================
// IMU
// =============================================================================
#define IMU_POLLING_INTERVAL_MS       4    // SHTP drain gate — matched to the rotation report interval so polls are never empty (empty polls cost real loop time, benchmarks.md #4)
#define IMU_REPORT_INTERVAL_MS        4    // rotation-vector report interval — sensor rounds to its supported grid and actually delivers ~370 Hz (measured 2026-07-16); request 5 for a true 200 Hz
#define IMU_ACCEL_REPORT_INTERVAL_MS 20    // linear-accel report interval (50 Hz) — only feeds the 50 Hz telemetry lax field; keeping it slow keeps SHTP drains short enough to fit the encoder's poll window
#define IMU_STALL_WARN_MS           400    // report a rotation-vector dropout this long as a stall (above the ~300ms WiFi-burst loop stall, so it flags a dead sensor not a stalled loop); the 1s reinit stays the recovery escalation
// #define IMU_ACCEL_PLOTTER           // Uncomment to stream lax/lay/laz to serial plotter

// =============================================================================
// UWB — Makerfabs DW3000 AOA anchor (Qorvo DW3000, STM32F103C8T6)
// =============================================================================
#define DW3000_BAUD            115200
// Multiply raw int32 angle field by this to get degrees.
// Set to 0.01f if firmware reports degrees*100 (verify from first-flash log).
#define DW3000_ANGLE_SCALE     1.0f
#define UWB_OUTLIER_REJECT_CM 30.0f  // Reject single-frame distance jumps larger than this
#define UWB_OUTLIER_MAX_STREAK    3  // Force-accept after this many consecutive rejections

// =============================================================================
// RPM — hall-effect interrupt (speed) + AS5600 encoder (odometry + cogging detection)
// =============================================================================

// Hall-effect sensor (speed)
#define RPM_STALE_MS             250       // zero speed if no pulse received within this window
#define RPM_PULSES_PER_REV       2         // hall-effect pulses per motor shaft revolution
#define RPM_HALL_SPEED_FACTOR    0.002017f // mph per motor RPM — retune if top speed reads wrong
#define RPM_HALL_CM_PER_PULSE    (RPM_HALL_SPEED_FACTOR * 2682.24f / RPM_PULSES_PER_REV) // cm of wheel travel per hall pulse (~2.7 cm)
// ISR debounce window is speed-adaptive: a fraction of the real pulse period at the current fused
// speed — wide when slow (a fast repeat is surely bounce), tight when fast (never drop the closely-
// spaced next real pulse), clamped both ends. The main loop sizes it; the ISR reads it.
#define RPM_DEBOUNCE_SPEED_FACTOR 0.5f     // window = this × real pulse period → rejects edges implying >2× the current speed
#define RPM_DEBOUNCE_MIN_US       1500     // floor — never shorter than this (chatter is sub-ms; keeps margin at high speed)
#define RPM_DEBOUNCE_MAX_US      30000     // ceiling — window at rest (reaches ~1 mph); larger catches slower chatter but risks dropping a hard-launch pulse
#define RPM_HALL_MAX_ACCEL_MPH_S 33.0f     // reject a hall reading implying a larger |Δspeed/Δt| than this (~1.5 g) — a double-count/glitch implies hundreds of mph/s, far above any real accel/decel
// Phantom-odom gate (odometry only — speed is never gated, it fails high). Reject odom for a
// batch only when the encoder corroborates a still wheel: implied speed exceeds both the floor
// and this multiple of the fused estimate while the encoder reads stopped.
#define RPM_PHANTOM_REJECT_FACTOR 2.0f     // reject odom batch if implied speed exceeds this multiple of the fused estimate
#define RPM_PHANTOM_FLOOR_MPH     0.5f     // ...and exceeds this floor (fused ≈ 0 at rest would otherwise reject slow real creep)

// AS5600 encoder (odometry + cogging detection)
#define AS5600_ADDR              0x36      // I2C address (fixed in hardware)
#define AS5600_COUNTS_PER_REV    4096      // 12-bit absolute encoder resolution
#define RPM_POLL_INTERVAL_MS     4         // poll AS5600 at 250 Hz
#define RPM_CM_PER_COUNT         0.000616f // cm of wheel travel per AS5600 encoder count
#define RPM_ALIAS_FWD_MIN_MPH    2.0f      // above this (signed) fused speed, de-alias the encoder delta toward forward instead of blind shortest-path
// Fused speed — encoder R inflates exponentially across the ramp, then the measurement is
// skipped. De-aliasing (RPM_ALIAS_FWD_MIN_MPH) extends the encoder's reliable range past its raw
// ~7 mph alias limit at 250 Hz, so the ramp trusts it up to 15 mph.
#define RPM_FUSED_ENC_R              0.0794f   // encoder measurement noise at low speed — dashboard-tuned 2026-07-16
#define RPM_FUSED_HALL_R             0.2f      // hall measurement noise — coarser sensor (~2.7 cm/pulse), starting guess
#define RPM_FUSED_ENC_RAMP_START_MPH 10.0f     // encoder R inflation begins
#define RPM_FUSED_ENC_RAMP_END_MPH   15.0f     // encoder fully gated out of the fused filter above this
#define RPM_FUSED_ENC_R_DECADES      6.0f      // decades of R inflation across the ramp — KF weight ~ 1/R, so fade in decades, not linearly

// 2-state fused speed — [speed, accelBias], predicted by IMU forward accel (lax) at
// 50 Hz, corrected by the encoder (ramped R above) and hall.
#define RPM_FUSED2_Q_SPEED           0.0008f   // speed process noise per 20ms predict — raised to reduce fused lag
#define RPM_FUSED2_Q_BIAS            0.000001f // accel-bias random walk per predict — starting guess
#define RPM_COGGING_FREQ_HZ      30.0f    // measured cogging oscillation frequency (bench test)
#define RPM_COGGING_CYCLE_SAMPLES 8       // samples per cogging cycle: 1000 / (FREQ_HZ * POLL_MS)
#define RPM_COGGING_WINDOW       (RPM_COGGING_CYCLE_SAMPLES * 3)  // analysis window: ~3 full cycles
#define RPM_COGGING_MIN_SIGN_CHANGES 4     // direction reversals required to flag cogging
#define RPM_COGGING_MAX_NET_COUNTS   1623  // max net displacement (counts) while flagging cogging (~1cm)
#define RPM_COGGING_ENC_CLEAR_MPH  0.25f  // encoder EMA above this = net forward motion, not cogging
#define RPM_COGGING_ENC_EMA_ALPHA  0.15f  // EMA alpha for encoder velocity used in state machine
#define RPM_COGGING_MAX_SPEED_MPH  2.0f   // disable cogging detection above this hall-effect speed

// =============================================================================
// OLED
// =============================================================================
#define OLED_WIDTH              128
#define OLED_HEIGHT              64
#define OLED_ADDR              0x3C
#define OLED_UPDATE_INTERVAL_MS 200
#define OLED_WIFI_SPLASH_MS   10000  // how long the boot splash holds after WiFi comes up (shows SSID + dashboard IP)

// =============================================================================
// Follow behavior — tunables the Pi-side follow port will consume (no onboard consumer
// since FOLLOW_ME was removed; they still feed rtConfig + dashboard sliders)
// =============================================================================
#define FOLLOW_DISTANCE_CM  200.0f // Distance at which car stops
#define MAX_DISTANCE_CM     800.0f // Distance at which car runs at max speed
#define MIN_SPEED_MPH 1.0f  // speed when tag is just past follow distance
#define MAX_SPEED_MPH 5.0f  // speed when tag is at or beyond max distance

// =============================================================================
// Control — mode + throttle/steering PIDs
// =============================================================================
#define DEFAULT_CONTROL_MODE        ControlMode::SETPOINT  // mode set at end of setup(); dashboard /mode changes it after that
#define CONTROL_UPDATE_INTERVAL_MS  20     // PID update rate (50 Hz)
#define THROTTLE_PID_KP             0.5f   // bench-tuned 2026-07-15 on the stand (anti-windup PID, integrator-led)
#define THROTTLE_PID_KI             0.6f
#define THROTTLE_PID_KD             0.15f  // spikes on hall-speed quantization steps — set to 0 if throttle chatter matters
#define THROTTLE_D_FILTER_ALPHA     0.4f   // low-pass on the speed feeding ONLY the throttle D-term (1 = off, lower = smoother); P and I use the raw fused speed — tames the KD spikes above
#define THROTTLE_PID_BRAKE_LIMIT   -0.25f  // PID output floor: allow up to 25% active braking (negative → brake via actuators); 0.0 = coast-only
#define STEERING_PID_KP             0.012f  // ≈ 1/90°: maps ±90° error to ±1.0 steering
#define STEERING_PID_KI             0.006f
#define STEERING_PID_KD             0.003f
#define CMD_TIMEOUT_MS              300    // SETPOINT failsafe: ms without a valid serial command frame before neutral throttle (steering stays on, holding the last heading)

// =============================================================================
// Actuators — ESC + steering servo output shaping
// =============================================================================
#define THROTTLE_SCALE        0.275f // Max throttle (0.0–1.0) — bench-tuned 2026-07-15
#define THROTTLE_Deadband     0.08f  // Minimum throttle before movement (ESC threshold = 1540µs)
#define THROTTLE_SMOOTH_ALPHA 0.05f  // Exponential smoothing on throttle output (0=frozen, 1=no smoothing)
#define STEERING_MAX          0.9f   // Max steering output (0.0–1.0) — caps servo deflection to prevent brownout
#define STEERING_TRIM         0.0f   // Steering center left offset.
#define PWM_NEUTRAL_US 1500
#define PWM_MIN_US     1000
#define PWM_MAX_US     2000

// =============================================================================
// Pan servo — aims the UWB anchor at the tag (AOA quality degrades past ~±60°).
// The ESP32 only maps µs↔degrees and reports the angle; the Pi's tf tree does
// the frame correction. Calibrate the map with the pan-cal env (see pan.cpp).
// =============================================================================
#define PAN_SERVO_MIN_US         800   // travel endpoint, sweep-verified 2026-07-14
#define PAN_SERVO_MAX_US        2200   // travel endpoint, sweep-verified 2026-07-14
#define PAN_SERVO_CENTER_US     1500   // nominal pulse width for anchor boresight = car forward
#define PAN_SERVO_TRIM_US         80   // mechanical centering offset (µs), added to center — re-derive with a post-±30%-span cal run (last fit's trim was bent by nonlinear edge points)
#define PAN_SERVO_US_PER_DEG  -10.52f  // signed: pan deg (+right) = (us - center - trim) / this — pan-cal 2026-07-14 (two runs agree ~10.5–11.7 after remount)
#define PAN_SLEW_DEG_PER_S     50.0f   // max pan slew rate — µs step per 20ms tick derived from PAN_SERVO_US_PER_DEG, so this stays true across recalibrations
#define PAN_MAX_DEG            55.0f   // symmetric target clamp: one limit both sides for the Pi's benefit, ≤ the shorter side's physical travel at any plausible trim

// =============================================================================
// Misc
// =============================================================================
#define DASHBOARD_UPDATE_INTERVAL_MS 100
