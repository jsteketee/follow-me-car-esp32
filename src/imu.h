#pragma once
#include <stdint.h>

struct ImuData {
    float yaw;
    float pitch;
    float roll;
    float yawRate;         // deg/s, derived from consecutive rotation vector readings
    float ax, ay, az;
    float lax, lay, laz;  // linear acceleration (gravity removed), m/s²
    float gx, gy, gz;
    uint8_t  cal_rot;
    uint8_t  cal_accel;
    uint8_t  cal_gyro;
    uint32_t update_hz;
    float    latency_us;
    uint32_t timestamp;    // millis() of the last rotation-vector report; 0 until the first — feeds staleness flags
};

void            imu_init();
void            imu_update();
const ImuData&  imu_get();
