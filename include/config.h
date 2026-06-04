#pragma once

// =============================================================================
// Pin Assignments
// =============================================================================

// I2C
#define PIN_SDA               8
#define PIN_SCL               9

// UWB UART
#define PIN_UWB_LEFT_TX      17
#define PIN_UWB_LEFT_RX      18
#define PIN_UWB_RIGHT_TX     38
#define PIN_UWB_RIGHT_RX     39
#define PIN_UWB_FRONT_TX     15
#define PIN_UWB_FRONT_RX     16
#define PIN_UWB_NRST         21

// Camera I2C (Wire1 — separate bus from OLED/IMU)
#define PIN_CAMERA_SDA        4
#define PIN_CAMERA_SCL        5

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

// =============================================================================
// UWB
// =============================================================================
#define UWB_TAG_ADDRESS             "TAG"
#define UWB_RESPONSE_TIMEOUT_MS      100   // 2.5x typical ~75ms response time
#define UWB_POLL_INTERVAL_MS           0   // Start next cycle immediately after previous completes
#define UWB_CAL_POLL_INTERVAL_MS     250   // Poll interval during calibration
#define UWB_ANCHOR_SEPARATION_CM    36.0f  // Physical separation between left and right anchors
#define UWB_CALIBRATION_DISTANCE_CM 185.0f  // Known distance used during startup calibration
#define UWB_CALIBRATION_SAMPLES       20   // Number of ranging samples per anchor for calibration
#define UWB_CALIBRATE_ON_STARTUP    false  // Run anchor calibration on startup
#define UWB_DIAGNOSTICS_ON_STARTUP  false  // Run 30s per-anchor diagnostic test on startup
#define UWB_KALMAN_Q                 8.0f  // Process noise
#define UWB_KALMAN_R                75.0f  // Measurement noise variance
#define UWB_OUTLIER_REJECT_CM       30.0f  // Reject single-poll distance jumps larger than this
#define UWB_OUTLIER_MAX_STREAK          3  // Force-accept after this many consecutive rejections
#define UWB_STALE_HEADING_MS         1500  // Age after which heading is stale; outlier rejection is bypassed
#define UWB_FRONT_X_CM              -2.0f  // lateral position of front anchor (right-positive; left = negative)
#define UWB_FRONT_Y_CM              15.3f  // forward position of front anchor from left/right anchor line
#define UWB_FRONT_FLIP_ABRUPT_DEG  90.0f  // heading change above this is considered abrupt and requires streak confirmation
#define UWB_FRONT_FLIP_CONFIRM         3  // consecutive abrupt-flip readings required before latch flips

// =============================================================================
// Control
// =============================================================================
#define THROTTLE_SCALE      0.18f  // Max throttle (0.0–1.0)
#define THROTTLE_Deadband   0.0f  // Minimum throttle before movement
#define FOLLOW_DISTANCE_CM  200.0f // Distance at which throttle is 0
#define MAX_DISTANCE_CM     700.0f // Distance at which throttle is max
#define DEFAULT_NAV_MODE       NavMode::STOPPED
#define FIX_TIMEOUT_MS 1500  // Time without valid fix before disabling throttle

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
#define THROTTLE_PID_TARGET_MPH     2.0f   // target follow speed
#define THROTTLE_PID_KP             4.0f
#define THROTTLE_PID_KI             0.5f
#define THROTTLE_PID_KD             0.0f

#define THROTTLE_FF_K               0.0f   // Feed-forward: maps targetSpeedMph → throttle (normalized 0–1). Tune until FF alone holds ~target speed at steady state.
#define STEERING_MAX                0.65f   // Max steering output (0.0–1.0) — caps servo deflection to prevent brownout
#define STEERING_PID_KP             0.01f  // ≈ 1/90°: maps ±90° error to ±1.0 steering
#define STEERING_PID_KI             0.004f
#define STEERING_PID_KD             0.002f

// =============================================================================
// Misc
// =============================================================================
#define SERIAL_REPORT_INTERVAL_MS 100
