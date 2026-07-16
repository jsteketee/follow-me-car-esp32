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
static RateGate _gate{ IMU_POLLING_INTERVAL_MS };

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
    bno.enableReport(SH2_LINEAR_ACCELERATION, IMU_ACCEL_REPORT_INTERVAL_MS * 1000);
    // bno.enableReport(SH2_ACCELEROMETER, IMU_REPORT_INTERVAL_MS * 1000);
    // bno.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_REPORT_INTERVAL_MS * 1000);
}

void imu_init()
{
    // The BNO085 boots slower than the ESP32 and its SH-2 I2C interface doesn't
    // respond while still starting up, so a single begin attempt can lose the race
    // and fail "randomly". Retry a few times before giving up; if all attempts fail,
    // imu_update() keeps retrying at runtime instead of staying dead.
    for (int attempt = 1; !bno.begin_I2C(0x4B); attempt++) {
        if (attempt >= 5) {
            Serial.printf("[%s] ❌ BNO085 not found after %d attempts — will keep retrying from imu_update\n",
                          TAG, attempt);
            return;
        }
        Serial.printf("[%s] ⚠️ BNO085 begin failed (attempt %d/5) — retrying\n", TAG, attempt);
        delay(250);
    }
    enableReports();
    sh2_setDcdAutoSave(true);

    // Poll until the first rotation vector event arrives (or 3s timeout).
    // This consumes the power-on reset event so imu_update() never sees a false-positive
    // reset warning, and gives control_init() a valid initial yaw to seed the held heading.
    uint32_t deadline = millis() + 3000;
    while (millis() < deadline) {
        if (bno.getSensorEvent(&_sensorEvent) &&
            _sensorEvent.sensorId == SH2_ROTATION_VECTOR) {
            quaternionToEuler(
                _sensorEvent.un.rotationVector.real,
                _sensorEvent.un.rotationVector.i,
                _sensorEvent.un.rotationVector.j,
                _sensorEvent.un.rotationVector.k);
            _imuData.cal_rot = _sensorEvent.status & 0x03;
            // Consume the power-on reset flag so imu_update() doesn't log a spurious reset warning.
            bno.wasReset();
            imu_ready = true;
            Serial.printf("[%s] ✅ BNO085 ready  yaw=%.1f°  cal=%d/3\n", TAG, _imuData.yaw, _imuData.cal_rot);
            return;
        }
        delay(1);
    }
    // Consume the power-on reset flag so imu_update() doesn't log a spurious reset warning.
    bno.wasReset();
    imu_ready = true;
    Serial.printf("[%s] ⚠️ BNO085 init timeout — no rotation vector in 3s, yaw will be invalid\n", TAG);
}

// Need to figure this out.
void imu_update()
{
    // Boot-failure recovery: begin_I2C never succeeded, so retry a full init every 2s
    // instead of staying dead until a power cycle. begin_I2C blocks for a while when
    // the chip is absent, but with no gyro the car is flying blind anyway.
    if (!imu_ready) {
        static uint32_t _lastBeginRetryMs = 0;
        if (millis() - _lastBeginRetryMs < 2000) return;
        _lastBeginRetryMs = millis();
        if (bno.begin_I2C(0x4B)) {
            enableReports();
            sh2_setDcdAutoSave(true);
            bno.wasReset();
            imu_ready = true;
            ESP_LOGW(TAG, "✅ BNO085 recovered after failed init");
        }
        return;
    }
    float dt;
    if (!_gate.tick(dt)) return;

    if (bno.wasReset())
    {
        ESP_LOGW(TAG, "⚠️ BNO085 reset, re-enabling reports");
        enableReports();
    }

    // Dead-stream recovery: reports stopped for >1s with no reset event — the SH-2
    // session is wedged. Re-run the init handshake, rate-limited to every 2s so a
    // truly dead chip doesn't turn the loop into a string of blocking begins.
    if (_imuData.timestamp != 0 && millis() - _imuData.timestamp > 1000) {
        static uint32_t _lastReinitMs = 0;
        if (millis() - _lastReinitMs >= 2000) {
            _lastReinitMs = millis();
            ESP_LOGW(TAG, "⚠️ BNO085 stream dead >1s — reinitializing");
            if (bno.begin_I2C(0x4B)) {
                enableReports();
                bno.wasReset();
                ESP_LOGW(TAG, "✅ BNO085 reinitialized");
            }
        }
    }

    int eventsRead    = 0;
    int rotationsRead = 0;  // rotation-vector reports this drain — feeds update_hz so it counts data, not polls
    while (eventsRead < 10 && bno.getSensorEvent(&_sensorEvent)) {
        eventsRead++;
        switch (_sensorEvent.sensorId) {
        case SH2_ROTATION_VECTOR: {
            rotationsRead++;
            quaternionToEuler(
                _sensorEvent.un.rotationVector.real,
                _sensorEvent.un.rotationVector.i,
                _sensorEvent.un.rotationVector.j,
                _sensorEvent.un.rotationVector.k);
            _imuData.cal_rot = _sensorEvent.status & 0x03;
            static float    _prevYaw   = NAN;
            static uint32_t _prevYawMs = 0;
            uint32_t nowMsYaw = millis();
            if (!isnan(_prevYaw)) {
                float delta = _imuData.yaw - _prevYaw;
                // Unwrap 0–360° range
                if (delta >  180.0f) delta -= 360.0f;
                if (delta < -180.0f) delta += 360.0f;
                float yawDt = (nowMsYaw - _prevYawMs) / 1000.0f;
                if (yawDt > 0.0f && yawDt < 0.5f)
                    _imuData.yawRate = delta / yawDt;
            }
            _prevYaw   = _imuData.yaw;
            _prevYawMs = nowMsYaw;
            _imuData.timestamp = nowMsYaw;
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
            if (lastRotationUs != 0) {
                if (interval < minIntervalUs) minIntervalUs = interval;
                if (interval > maxIntervalUs) maxIntervalUs = interval;
                _imuData.latency_us = 0.9f * _imuData.latency_us + 0.1f * interval;
            }
            lastRotationUs = nowUs;
            uint32_t nowMs = millis();
            if (nowMs - lastReportMs >= 1000) {
                lastReportMs = nowMs;
                ESP_LOGD(TAG, "rotation interval: avg=%uus  min=%uus  max=%uus  jitter=%uus",
                    (uint32_t)_imuData.latency_us,
                    minIntervalUs == UINT32_MAX ? 0 : minIntervalUs,
                    maxIntervalUs,
                    maxIntervalUs - (minIntervalUs == UINT32_MAX ? 0 : minIntervalUs));
                minIntervalUs = UINT32_MAX;
                maxIntervalUs = 0;
            }
            break;
        }
        case SH2_LINEAR_ACCELERATION:
            _imuData.lax = _sensorEvent.un.linearAcceleration.x;
            _imuData.lay = _sensorEvent.un.linearAcceleration.y;
            _imuData.laz = _sensorEvent.un.linearAcceleration.z;
            _imuData.cal_accel = _sensorEvent.status & 0x03;
            #ifdef IMU_ACCEL_PLOTTER
            Serial.printf("lax:%.3f,lay:%.3f,laz:%.3f\n", _imuData.lax, _imuData.lay, _imuData.laz);
            #endif
            break;
        case SH2_ACCELEROMETER:
            _imuData.ax = _sensorEvent.un.accelerometer.x;
            _imuData.ay = _sensorEvent.un.accelerometer.y;
            _imuData.az = _sensorEvent.un.accelerometer.z;
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

    // Rate of actual rotation reports (not gate ticks) — calling with 0 on empty
    // drains keeps the rate decaying toward 0 if the sensor stops reporting.
    _imuHz.update(rotationsRead);
    _imuData.update_hz = (uint32_t)_imuHz.hz;
}

const ImuData &imu_get() { return _imuData; }
