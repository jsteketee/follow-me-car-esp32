#pragma once

// I2C
#define PIN_SDA 8
#define PIN_SCL 9

// UART — UWB anchors
#define PIN_UWB_LEFT_TX  43
#define PIN_UWB_LEFT_RX  44
#define PIN_UWB_RIGHT_TX 17
#define PIN_UWB_RIGHT_RX 18
#define PIN_UWB_REAR_TX  15
#define PIN_UWB_REAR_RX  16

// PWM inputs (via logic shifter)
#define PIN_PWM_THROTTLE_IN 4
#define PIN_PWM_STEER_IN    5
#define PIN_PWM_MODE_IN     6

// PWM outputs
#define PIN_ESC  1
#define PIN_SERVO 2

// Misc
#define PIN_BUZZER 12

// OLED
#define OLED_WIDTH              128
#define OLED_HEIGHT             64
#define OLED_ADDR               0x3C

//Update Intervals in milliseconds
#define OLED_UPDATE_INTERVAL_MS 100
#define SERIAL_REPORT_INTERVAL_MS 100
#define IMU_POLLING_INTERVAL_MS  40 //How often IMU is polled
#define IMU_REPORT_INTERVAL_MS  50//How often IMU reports data
