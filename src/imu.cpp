// BNO085 IMU driver over I2C. Publishes yaw/pitch/roll, accel, gyro, and per-sensor calibration status.
#include "imu.h"
#include "config.h"
#include "utils.h"
#include <Adafruit_BNO08x.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "imu";
static HzTracker _imuHz;

static Adafruit_BNO08x bno;
static bool imu_ready = false;
static sh2_SensorValue_t _sensorEvent;
static ImuData _imuData = {};

static void quaternionToEuler(float qw, float qx, float qy, float qz)
{
    float sinr = 2.0f * (qw * qx + qy * qz);
    float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
    _imuData.roll = atan2f(sinr, cosr) * 180.0f / M_PI;

    float sinp = 2.0f * (qw * qy - qz * qx);
    _imuData.pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp) : asinf(sinp) * 180.0f / M_PI;

    float siny = 2.0f * (qw * qz + qx * qy);
    float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
    _imuData.yaw = atan2f(siny, cosy) * 180.0f / M_PI;
    if (_imuData.yaw < 0)
        _imuData.yaw += 360.0f;
}

static void enableReports()
{
    bno.enableReport(SH2_ROTATION_VECTOR, IMU_REPORT_INTERVAL_MS * 1000);
    // bno.enableReport(SH2_ACCELEROMETER, IMU_REPORT_INTERVAL_MS * 1000);
    // bno.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_REPORT_INTERVAL_MS * 1000);
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
    _imuHz.update();
    _imuData.update_hz = (uint32_t)_imuHz.hz;

    if (bno.wasReset())
    {
        ESP_LOGW(TAG, "⚠️ BNO085 reset, re-enabling reports");
        enableReports();
    }

    int eventsRead = 0;
    while (eventsRead < 10 && bno.getSensorEvent(&_sensorEvent)) {
        eventsRead++;
        switch (_sensorEvent.sensorId) {
        case SH2_ROTATION_VECTOR: {
            quaternionToEuler(
                _sensorEvent.un.rotationVector.real,
                _sensorEvent.un.rotationVector.i,
                _sensorEvent.un.rotationVector.j,
                _sensorEvent.un.rotationVector.k);
            _imuData.cal_rot = _sensorEvent.status & 0x03;
            static bool headingCalLogged = false;
            if (!headingCalLogged && _imuData.cal_rot == 3) {
                ESP_LOGI(TAG, "✅ Heading calibrated");
                headingCalLogged = true;
            }
            static uint32_t lastRotationUs = 0;
            static uint32_t minIntervalUs  = UINT32_MAX;
            static uint32_t maxIntervalUs  = 0;
            static uint32_t lastReportMs   = 0;
            uint32_t nowUs    = micros();
            uint32_t interval = nowUs - lastRotationUs;
            lastRotationUs    = nowUs;
            if (lastRotationUs != 0) {
                if (interval < minIntervalUs) minIntervalUs = interval;
                if (interval > maxIntervalUs) maxIntervalUs = interval;
            }
            _imuData.latency_us = 0.9f * _imuData.latency_us + 0.1f * interval;
            uint32_t nowMs = millis();
            if (nowMs - lastReportMs >= 1000) {
                lastReportMs = nowMs;
                ESP_LOGI(TAG, "rotation interval: avg=%uus  min=%uus  max=%uus  jitter=%uus",
                    (uint32_t)_imuData.latency_us,
                    minIntervalUs == UINT32_MAX ? 0 : minIntervalUs,
                    maxIntervalUs,
                    maxIntervalUs - (minIntervalUs == UINT32_MAX ? 0 : minIntervalUs));
                minIntervalUs = UINT32_MAX;
                maxIntervalUs = 0;
            }
            break;
        }
        case SH2_ACCELEROMETER:
            _imuData.ax = _sensorEvent.un.accelerometer.x;
            _imuData.ay = _sensorEvent.un.accelerometer.y;
            _imuData.az = _sensorEvent.un.accelerometer.z;
            _imuData.cal_accel = _sensorEvent.status & 0x03;
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            _imuData.gx = _sensorEvent.un.gyroscope.x;
            _imuData.gy = _sensorEvent.un.gyroscope.y;
            _imuData.gz = _sensorEvent.un.gyroscope.z;
            _imuData.cal_gyro = _sensorEvent.status & 0x03;
            break;
        }
    }
    if (eventsRead == 10)
        ESP_LOGW(TAG, "⚠️ IMU sensor buffer hit cap");
}

const ImuData &imu_get() { return _imuData; }
