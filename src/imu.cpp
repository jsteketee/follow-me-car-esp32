// BNO085 IMU driver over I2C. Publishes yaw/pitch/roll, accel, gyro, and per-sensor calibration status.
#include "imu.h"
#include "config.h"
#include "utils.h"
#include <Adafruit_BNO08x.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "imu";
static HzTracker imuHz;

static Adafruit_BNO08x bno;
static bool imu_ready = false;
static sh2_SensorValue_t sensorValue;
static ImuData _data = {};

static void quaternionToEuler(float qw, float qx, float qy, float qz)
{
    float sinr = 2.0f * (qw * qx + qy * qz);
    float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
    _data.roll = atan2f(sinr, cosr) * 180.0f / M_PI;

    float sinp = 2.0f * (qw * qy - qz * qx);
    _data.pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp) : asinf(sinp) * 180.0f / M_PI;

    float siny = 2.0f * (qw * qz + qx * qy);
    float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
    _data.yaw = atan2f(siny, cosy) * 180.0f / M_PI;
    if (_data.yaw < 0)
        _data.yaw += 360.0f;
}

static void enableReports()
{
    bno.enableReport(SH2_ROTATION_VECTOR, IMU_REPORT_INTERVAL_MS * 1000);
    bno.enableReport(SH2_ACCELEROMETER, IMU_REPORT_INTERVAL_MS * 1000);
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_REPORT_INTERVAL_MS * 1000);
}

void imu_init()
{
    if (!bno.begin_I2C(0x4B))
    {
        ESP_LOGE(TAG, "❌ BNO085 not found");
        return;
    }
    enableReports();
    imu_ready = true;
    ESP_LOGI(TAG, "✅ BNO085 ready");
}

// Need to figure this out.
void imu_update()
{
    // Exit early if IMU isn't ready, or if it's not time for an update yet
    if (!imu_ready)
        return;
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    if (now - lastUpdate < IMU_POLLING_INTERVAL_MS)
        return;
    lastUpdate = now;

    // Update and expose the IMU update rate tracker
    imuHz.update();
    _data.update_hz = (uint32_t)imuHz.hz;

    if (bno.wasReset())
    {
        ESP_LOGW(TAG, "⚠️ BNO085 reset, re-enabling reports");
        enableReports();
    }

    int eventsRead = 0;
    while (eventsRead < 10 && bno.getSensorEvent(&sensorValue)) {
        eventsRead++;
        switch (sensorValue.sensorId) {
        case SH2_ROTATION_VECTOR: {
            quaternionToEuler(
                sensorValue.un.rotationVector.real,
                sensorValue.un.rotationVector.i,
                sensorValue.un.rotationVector.j,
                sensorValue.un.rotationVector.k);
            _data.cal_rot = sensorValue.status & 0x03;
            static bool headingCalLogged = false;
            if (!headingCalLogged && _data.cal_rot == 3) {
                ESP_LOGI(TAG, "✅ Heading calibrated");
                headingCalLogged = true;
            }
            static uint32_t lastRotationUs = 0;
            uint32_t nowUs = micros();
            _data.latency_us = 0.9f * _data.latency_us + 0.1f * (nowUs - lastRotationUs);
            lastRotationUs = nowUs;
            ESP_LOGD(TAG, "latency_us:%u", (uint32_t)_data.latency_us);
            break;
        }
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
    if (eventsRead == 10)
        ESP_LOGW(TAG, "⚠️ IMU sensor buffer hit cap");
}

const ImuData &imu_get() { return _data; }
