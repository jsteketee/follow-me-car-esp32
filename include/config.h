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
#define PIN_UWB_REAR_TX      15
#define PIN_UWB_REAR_RX      16
#define PIN_UWB_NRST         21

// PWM inputs (via logic shifter)
#define PIN_PWM_THROTTLE_IN   4
#define PIN_PWM_STEER_IN      5
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
#define WIFI_AP_SSID     "followme-car"
#define WIFI_AP_PASS     ""
#define WIFI_HOSTNAME    "followme-car"
#define TELNET_PORT      23

// =============================================================================
// OLED
// =============================================================================
#define OLED_WIDTH              128
#define OLED_HEIGHT              64
#define OLED_ADDR              0x3C
#define OLED_UPDATE_INTERVAL_MS 100

// =============================================================================
// IMU
// =============================================================================
#define IMU_POLLING_INTERVAL_MS  40   // How often IMU is polled
#define IMU_REPORT_INTERVAL_MS   50   // How often IMU reports data

// =============================================================================
// UWB
// =============================================================================
#define UWB_TAG_ADDRESS             "TAG"
#define UWB_RESPONSE_TIMEOUT_MS      500
#define UWB_POLL_INTERVAL_MS          50   // How often anchors are polled
#define UWB_ANCHOR_STAGGER_MS         25   // Delay between firing left and right anchors
#define UWB_CAL_POLL_INTERVAL_MS     250   // Poll interval during calibration
#define UWB_ANCHOR_SEPARATION_CM    36.0f  // Physical separation between left and right anchors
#define UWB_CALIBRATION_DISTANCE_CM 70.0f  // Known distance used during startup calibration
#define UWB_CALIBRATION_SAMPLES       20   // Number of ranging samples per anchor for calibration
#define UWB_CALIBRATE_ON_STARTUP    false  // Run anchor calibration on startup
#define UWB_DIAGNOSTICS_ON_STARTUP  false  // Run 30s per-anchor diagnostic test on startup
#define UWB_KALMAN_Q                25.0f  // Process noise
#define UWB_KALMAN_R                35.0f  // Measurement noise variance
#define UWB_OUTLIER_REJECT_CM       30.0f  // Reject single-poll distance jumps larger than this
#define UWB_OUTLIER_MAX_STREAK          3  // Force-accept after this many consecutive rejections
#define UWB_STALE_HEADING_MS         1500  // Age after which heading is stale; outlier rejection is bypassed

// =============================================================================
// Control
// =============================================================================
#define THROTTLE_SCALE      0.25f  // Max throttle (0.0–1.0)
#define THROTTLE_Deadband   0.15f  // Minimum throttle before movement
#define FOLLOW_DISTANCE_CM  200.0f // Distance at which throttle is 0
#define MAX_DISTANCE_CM     700.0f // Distance at which throttle is max

// =============================================================================
// Misc
// =============================================================================
#define SERIAL_REPORT_INTERVAL_MS 100
