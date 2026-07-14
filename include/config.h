#pragma once

// Ordered to follow the data flow: pins → comms → sensors → display → fusion →
// navigation → control → actuators.

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
#define IMU_POLLING_INTERVAL_MS  10   // How often IMU is polled
#define IMU_REPORT_INTERVAL_MS   10   // How often IMU reports data
// #define IMU_ACCEL_PLOTTER           // Uncomment to stream lax/lay/laz to serial plotter

// =============================================================================
// UWB — Makerfabs DW3000 AOA anchor (Qorvo DW3000, STM32F103C8T6)
// =============================================================================
#define DW3000_BAUD            115200
// Multiply raw int32 angle field by this to get degrees.
// Set to 0.01f if firmware reports degrees*100 (verify from first-flash log).
#define DW3000_ANGLE_SCALE     1.0f
#define UWB_KALMAN_Q           8.0f  // Distance filter process noise; ~3 cm/frame allows tracking at walking speed
#define UWB_KALMAN_R          20.0f  // Distance filter measurement noise; stationary test: σ≈4cm → σ²≈14-20 cm²
#define UWB_OUTLIER_REJECT_CM 30.0f  // Reject single-frame distance jumps larger than this
#define UWB_OUTLIER_MAX_STREAK    3  // Force-accept after this many consecutive rejections

// =============================================================================
// RPM — hall-effect interrupt (speed + odometry) + AS5600 encoder (cogging detection)
// =============================================================================

// Hall-effect sensor (speed + odometry)
#define RPM_STALE_MS             250       // zero speed if no pulse received within this window
#define RPM_PULSES_PER_REV       2         // hall-effect pulses per motor shaft revolution
#define RPM_HALL_SPEED_FACTOR    0.002017f // mph per motor RPM — retune if top speed reads wrong
#define RPM_HALL_CM_PER_PULSE    (RPM_HALL_SPEED_FACTOR * 2682.24f / RPM_PULSES_PER_REV) // cm of wheel travel per hall pulse (~2.7 cm)
#define RPM_EMA_ALPHA            0.3f      // EMA smoothing on speed output (0=frozen, 1=no smoothing)
#define RPM_SPIKE_REJECT_FACTOR  2.0f      // Reject pulse if instantaneous speed exceeds this multiple of filtered speed
#define RPM_SPIKE_MAX_STREAK     1         // Force-accept after this many consecutive rejections

// AS5600 encoder (cogging detection only)
#define AS5600_ADDR              0x36      // I2C address (fixed in hardware)
#define AS5600_COUNTS_PER_REV    4096      // 12-bit absolute encoder resolution
#define RPM_POLL_INTERVAL_MS     5         // poll AS5600 at 200 Hz
#define RPM_CM_PER_COUNT         0.000616f // cm of wheel travel per AS5600 encoder count
#define RPM_COGGING_FREQ_HZ      30.0f    // measured cogging oscillation frequency (bench test)
#define RPM_COGGING_CYCLE_SAMPLES 7       // samples per cogging cycle: 1000 / (FREQ_HZ * POLL_MS)
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
// Fusion
// =============================================================================
#define FUSION_SENSOR_TIMEOUT_SEC  3.0f   // seconds without a fix before uncertainty crosses the stale threshold
#define FUSION_KALMAN_R_UWB      15.0f  // UWB bearing measurement noise (deg²); stationary test: σ≈3-4° → σ²≈10-15
#define FUSION_INNOV_MEAN_ALPHA   0.4f   // how fast the innovation mean tracks genuine movement; higher = faster tracking, less sensitive to real motion
#define FUSION_INNOV_EWMA_ALPHA   0.15f  // innovation EWMA decay: higher = faster spike, faster recovery; lower = slower but smoother
#define FUSION_STALE_UNCERTAINTY 150.0f // uncertainty (deg²) above which nav treats the estimate as stale; steady state ~17, erratic movement ~120

// =============================================================================
// Follow behavior — tunables the Pi-side follow port will consume (no onboard consumer
// since FOLLOW_ME was removed; they still feed rtConfig + dashboard sliders)
// =============================================================================
#define FOLLOW_DISTANCE_CM  200.0f // Distance at which car stops
#define MAX_DISTANCE_CM     800.0f // Distance at which car runs at max speed
#define MIN_SPEED_MPH 1.0f  // speed when tag is just past follow distance
#define MAX_SPEED_MPH 2.5f  // speed when tag is at or beyond max distance

// =============================================================================
// Control — mode + throttle/steering PIDs
// =============================================================================
#define DEFAULT_CONTROL_MODE        ControlMode::REMOTE  // mode set at end of setup(); dashboard /mode changes it after that
#define CONTROL_UPDATE_INTERVAL_MS  20     // PID update rate (50 Hz)
#define THROTTLE_PID_KP             1.7f
#define THROTTLE_PID_KI             0.5f
#define THROTTLE_PID_KD             0.0f
#define STEERING_PID_KP             0.012f  // ≈ 1/90°: maps ±90° error to ±1.0 steering
#define STEERING_PID_KI             0.006f
#define STEERING_PID_KD             0.003f
#define CMD_TIMEOUT_MS              300    // REMOTE failsafe: ms without a valid serial command frame before neutral throttle (steering stays on, holding the last heading)

// =============================================================================
// Actuators — ESC + steering servo output shaping
// =============================================================================
#define THROTTLE_SCALE        0.25f  // Max throttle (0.0–1.0)
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
#define PAN_SERVO_TRIM_US         28   // mechanical centering offset (µs), added to center — re-derive with a post-±30%-span cal run (last fit's trim was bent by nonlinear edge points)
#define PAN_SERVO_US_PER_DEG  -10.52f  // signed: pan deg (+right) = (us - center - trim) / this — pan-cal 2026-07-14 (two runs agree ~10.5–11.7 after remount)
#define PAN_SLEW_DEG_PER_S     50.0f   // max pan slew rate — µs step per 20ms tick derived from PAN_SERVO_US_PER_DEG, so this stays true across recalibrations
#define PAN_MAX_DEG            55.0f   // symmetric target clamp: one limit both sides for the Pi's benefit, ≤ the shorter side's physical travel at any plausible trim

// =============================================================================
// Misc
// =============================================================================
#define SERIAL_REPORT_INTERVAL_MS    100
#define DASHBOARD_UPDATE_INTERVAL_MS 100
