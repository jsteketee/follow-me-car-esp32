#pragma once
#include <stdint.h>

struct ImuData {
    float yaw;
    float pitch;
    float roll;
    float ax, ay, az;
    float gx, gy, gz;
    uint8_t  cal_rot;
    uint8_t  cal_accel;
    uint8_t  cal_gyro;
    uint32_t update_hz;
};

void            imu_init();
void            imu_update();
const ImuData&  imu_get();
