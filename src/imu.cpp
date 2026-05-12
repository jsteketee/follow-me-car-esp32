#include "imu.h"
#include "config.h"
#include <Adafruit_BNO08x.h>
#include <math.h>

static Adafruit_BNO08x bno;
static bool imu_ready = false;
static sh2_SensorValue_t sensorValue;
static ImuData _data = {};

static void quaternionToEuler(float qw, float qx, float qy, float qz) {
    float sinr = 2.0f * (qw * qx + qy * qz);
    float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
    _data.roll = atan2f(sinr, cosr) * 180.0f / M_PI;

    float sinp = 2.0f * (qw * qy - qz * qx);
    _data.pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp) : asinf(sinp) * 180.0f / M_PI;

    float siny = 2.0f * (qw * qz + qx * qy);
    float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
    _data.yaw = atan2f(siny, cosy) * 180.0f / M_PI;
    if (_data.yaw < 0) _data.yaw += 360.0f;
}

static void enableReports() {
    bno.enableReport(SH2_ROTATION_VECTOR,     5000);
    bno.enableReport(SH2_ACCELEROMETER,        5000);
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);
}

void imu_init() {
    if (!bno.begin_I2C(0x4B)) {
        Serial.println("BNO085 not found");
        return;
    }
    enableReports();
    imu_ready = true;
    Serial.println("BNO085 ready");
}

//Need to figure this out. 
void imu_update() {
    if (!imu_ready) return;
    static uint32_t lastUpdate = 0;
    if (millis() - lastUpdate < IMU_UPDATE_INTERVAL_MS) return;
    lastUpdate = millis();

    static uint32_t callCount = 0;
    static uint32_t lastHz = 0;
    callCount++;
    if (millis() - lastHz >= 1000) {
        _data.update_hz = callCount;
        callCount = 0;
        lastHz = millis();
    }

    if (bno.wasReset()) {
        Serial.println("BNO085 reset, re-enabling reports");
        enableReports();
    }

    for (int i = 0; i < 10 && bno.getSensorEvent(&sensorValue); i++) {
        switch (sensorValue.sensorId) {
            case SH2_ROTATION_VECTOR:
                quaternionToEuler(
                    sensorValue.un.rotationVector.real,
                    sensorValue.un.rotationVector.i,
                    sensorValue.un.rotationVector.j,
                    sensorValue.un.rotationVector.k
                );
                _data.cal_rot = sensorValue.status & 0x03;
                break;
            case SH2_ACCELEROMETER:
                _data.ax = sensorValue.un.accelerometer.x;
                _data.ay = sensorValue.un.accelerometer.y;
                _data.az = sensorValue.un.accelerometer.z;
                _data.cal_accel = sensorValue.status & 0x03;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                _data.gx = sensorValue.un.gyroscope.x;
                _data.gy = sensorValue.un.gyroscope.y;
                _data.gz = sensorValue.un.gyroscope.z;
                _data.cal_gyro = sensorValue.status & 0x03;
                break;
        }
    }
}

const ImuData& imu_get() { return _data; }
