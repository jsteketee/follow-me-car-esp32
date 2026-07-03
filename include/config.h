#pragma once

// =============================================================================
// Pin Assignments
// =============================================================================

// I2C
#define PIN_SDA               8
#define PIN_SCL               9

// DW3000 UWB UART
#define PIN_DW3000_RX         4
#define PIN_DW3000_TX         5

// Camera I2C (shares Wire bus with OLED/IMU — pull-ups provided by those breakouts)
#define PIN_CAMERA_SDA        8
#define PIN_CAMERA_SCL        9

// PWM inputs (via logic shifter)
#define PIN_PWM_MODE_IN       6

// PWM outputs
#define PIN_ESC               1
#define PIN_SERVO             2

// Misc
#define PIN_BUZZER           12
#define PIN_LED              13
#define PIN_RPM              10

// =============================================================================
// WiFi
// =============================================================================
#include "secrets.h"
#define WIFI_AP_SSID     "Follow Me"
#define WIFI_AP_PASS     ""
#define WIFI_HOSTNAME    "Follow Me"
#define TELNET_PORT      23

// =============================================================================
// Camera
// =============================================================================
#define CAMERA_I2C_ADDR            0x42
#define CAMERA_UPDATE_INTERVAL_MS    40  // 25 Hz
#define CAMERA_H_FOV_DEG           66.0f // OV2640 horizontal field of view

// =============================================================================
// OLED
// =============================================================================
#define OLED_WIDTH              128
#define OLED_HEIGHT              64
#define OLED_ADDR              0x3C
#define OLED_UPDATE_INTERVAL_MS 100

// =============================================================================
// SENSORS
// =============================================================================

// =============================================================================
// IMU
// =============================================================================
#define IMU_POLLING_INTERVAL_MS  10   // How often IMU is polled
#define IMU_REPORT_INTERVAL_MS   10   // How often IMU reports data
// #define IMU_ACCEL_PLOTTER           // Uncomment to stream lax/lay/laz to serial plotter

// =============================================================================
// Cogging Detection
// =============================================================================
#define COGGING_LAY_MEAN_ALPHA    0.05f  // smoothing for lay mean (~200ms window — tracks net direction)
#define COGGING_LAY_VAR_ALPHA     0.10f  // smoothing for lay variance (~100ms window — tracks oscillation energy)
#define COGGING_MEAN_THRESHOLD    0.5f   // max |mean| (m/s²) to qualify as near-zero net acceleration
#define COGGING_VAR_THRESHOLD     4.0f   // min variance (m/s²)² to qualify as high-energy oscillation
#define COGGING_HOLD_MS           500    // minimum time to hold cogging flag after last detection

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
// Control
// =============================================================================
#define THROTTLE_SCALE      0.22f  // Max throttle (0.0–1.0)
#define THROTTLE_Deadband   0.05f  // Minimum throttle before movement
#define FOLLOW_DISTANCE_CM  200.0f // Distance at which car stops
#define MAX_DISTANCE_CM     800.0f // Distance at which car runs at max speed
#define MIN_SPEED_MPH 1.0f  // speed when tag is just past follow distance
#define MAX_SPEED_MPH 2.5f  // speed when tag is at or beyond max distance
#define DEFAULT_NAV_MODE       NavMode::FOLLOW_ME


// =============================================================================
// RPM
// =============================================================================
#define RPM_PULSES_PER_REV       1         // Hall effect pulses per motor shaft revolution
#define RPM_SPEED_FACTOR         0.002017f // mph per motor RPM — tune until displayed speed matches a known reference
#define RPM_STALE_MS             250       // Zero speed if no pulse received within this window
#define RPM_KALMAN_Q             0.6f      // Process noise — how fast true speed can change per step
#define RPM_KALMAN_R             0.3f      // Measurement noise — pulse period is clean so this is low
#define RPM_SPIKE_REJECT_FACTOR  2.0f      // Reject pulse if instantaneous speed exceeds this multiple of current filtered speed
#define RPM_SPIKE_MAX_STREAK     1         // Force-accept after this many consecutive rejections (prevents blocking genuine acceleration)

// =============================================================================
// Throttle PID
// =============================================================================
#define CONTROL_UPDATE_INTERVAL_MS  20     // PID update rate (50 Hz)
#define THROTTLE_SMOOTH_ALPHA       0.05f  // Exponential smoothing on throttle output (0=frozen, 1=no smoothing)
#define THROTTLE_PID_KP             4.0f
#define THROTTLE_PID_KI             0.5f
#define THROTTLE_PID_KD             0.0f
#define THROTTLE_FF_K               0.0f   // Feed-forward: maps targetSpeedMph → throttle (normalized 0–1). Tune until FF alone holds ~target speed at steady state.
#define STEERING_MAX                0.65f   // Max steering output (0.0–1.0) — caps servo deflection to prevent brownout
#define STEERING_PID_KP             0.015f  // ≈ 1/90°: maps ±90° error to ±1.0 steering
#define STEERING_PID_KI             0.004f
#define STEERING_PID_KD             0.002f

// =============================================================================
// Fusion
// =============================================================================
#define FUSION_SENSOR_TIMEOUT_SEC  3.0f   // seconds without a fix before uncertainty crosses the stale threshold
#define FUSION_KALMAN_R_UWB      15.0f  // UWB bearing measurement noise (deg²); stationary test: σ≈3-4° → σ²≈10-15
#define FUSION_KALMAN_R_CAMERA    4.0f  // camera bearing measurement noise (deg²) — lower = trust camera more
#define FUSION_INNOV_MEAN_ALPHA   0.4f   // how fast the innovation mean tracks genuine movement; higher = faster tracking, less sensitive to real motion
#define FUSION_INNOV_EWMA_ALPHA   0.15f  // innovation EWMA decay: higher = faster spike, faster recovery; lower = slower but smoother
#define FUSION_STALE_UNCERTAINTY 150.0f // uncertainty (deg²) above which nav treats the estimate as stale; steady state ~17, erratic movement ~120

// =============================================================================
// Misc
// =============================================================================
#define SERIAL_REPORT_INTERVAL_MS    100
#define DASHBOARD_UPDATE_INTERVAL_MS 100
