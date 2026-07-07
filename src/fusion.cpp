// Sensor fusion layer.
// Tracks the tag's absolute compass bearing, blending UWB AOA angle and camera blob
// angle via a 1-D Kalman filter on bearing. Converting back to relative angle using imu.yaw
// gives automatic rotation compensation without gyro integration or drift.
#include "fusion.h"
#include "uwb.h"
#include "camera.h"
#include "imu.h"
#include "rpm.h"
#include "config.h"
#include "runtime_config.h"
#include "utils.h"
#include <math.h>
#include "esp_log.h"

static const char* TAG = "fusion";

static Pose     _fusedPose          = { .fusedAngle = 0.0f, .uwbAngle = NAN, .camAngle = NAN, .distanceCm = -1.0f, .uwbDistCm = -1.0f };
static KalmanFilter  _distKalman;                    // 1-D Kalman filter tracking distance to tag (cm)
static float         _bearingDeg          = 0.0f;    // Kalman state estimate of absolute bearing to tag
static float         _bearingP            = 1000.0f; // bearing estimate variance (deg²)
static float         _innovEwma           = 0.0f;    // EWMA of innov² (deg²) — spikes on surprise, decays on consistent readings
static uint32_t      _lastUpdateMs        = 0;
static unsigned long _lastUwbTimestamp    = 0;
static unsigned long _lastCameraTimestamp = 0;
static float         _lastOdometryCm      = 0.0f;
static float         _fusedOdometryCm     = 0.0f;
static bool          _bearingSeeded       = false;

// Update the absolute bearing estimate from a new relative angle measurement
static void correct_bearing(float relAngle, float imuYaw, float r) {
    float measured = imuYaw - relAngle;
    while (measured >= 360.0f) measured -= 360.0f;
    while (measured <    0.0f) measured += 360.0f;

    float innov = measured - _bearingDeg;
    while (innov >  180.0f) innov -= 360.0f;
    while (innov < -180.0f) innov += 360.0f;

    // Variance-based erratic detector. A fast mean tracks genuine consistent movement,
    // so deviations around it stay small. Erratic readings scatter around the mean
    // and produce large deviations regardless of whether they flip sign.
    static float _innovMean = 0.0f;
    _innovMean += rtConfig.fusionInnovMeanAlpha * (innov - _innovMean);
    float deviation = innov - _innovMean;
    _innovEwma = fminf(rtConfig.fusionInnovEwmaAlpha * (deviation * deviation) + (1.0f - rtConfig.fusionInnovEwmaAlpha) * _innovEwma,
                       1.1f * rtConfig.fusionStaleUncertainty);

    float k     = _bearingP / (_bearingP + r);
    _bearingDeg += k * innov;
    while (_bearingDeg >= 360.0f) _bearingDeg -= 360.0f;
    while (_bearingDeg <    0.0f) _bearingDeg += 360.0f;
    _bearingP *= (1.0f - k);
}


void fusion_init() {
    // Seed the bearing from IMU heading captured during imu_init().
    // imu_init() blocks until the first rotation vector arrives, so yaw should be valid here.
    const ImuData& imu = imu_get();
    if (!isnan(imu.yaw)) {
        _bearingDeg   = imu.yaw;
        _bearingSeeded = true;
        ESP_LOGI(TAG, "✅ fusion ready  bearing seeded from IMU yaw=%.1f°  rUwb=%.1f  rCam=%.1f  stale=%.1f  innovMeanAlpha=%.3f  innovEwmaAlpha=%.3f",
            imu.yaw, rtConfig.fusionRUwb, rtConfig.fusionRCamera, rtConfig.fusionStaleUncertainty,
            rtConfig.fusionInnovMeanAlpha, rtConfig.fusionInnovEwmaAlpha);
    } else {
        ESP_LOGW(TAG, "✅ fusion ready  (no IMU yaw — bearing will seed on first update)  rUwb=%.1f  rCam=%.1f",
            rtConfig.fusionRUwb, rtConfig.fusionRCamera);
    }
}

bool fusion_update() {
    const ImuData&    imu = imu_get();
    const UWBReading& uwb = uwb_get();
    const CameraData& cam = camera_get();

    uint32_t now = millis();

    // Fallback seed in case fusion_init() didn't have a valid IMU yaw yet.
    if (!_bearingSeeded && !isnan(imu.yaw)) {
        _bearingDeg   = imu.yaw;
        _bearingSeeded = true;
    }

    // Grow bearing uncertainty every loop as data becomes stale.
    if (_lastUpdateMs > 0) {
        float dt = (now - _lastUpdateMs) / 1000.0f;
        _bearingP += (rtConfig.fusionStaleUncertainty / rtConfig.sensorTimeoutSec) * dt;
    }
    _lastUpdateMs = now;

    // Dead reckoning: update distance estimate from wheel odometry between UWB readings.
    float rpmOdom = rpm_get().odometryCm;
    float traveled = rpmOdom - _lastOdometryCm;
    _lastOdometryCm = rpmOdom;
    _fusedOdometryCm += traveled;
    if (_distKalman.initialized)
        _distKalman.x = fmaxf(0.0f, _distKalman.x - traveled);
    _fusedPose.distanceCm = _distKalman.initialized ? _distKalman.x : -1.0f;


    
    // Nudge bearing from UWB reading. The Kalman filter is applied in correct_bearing().
    bool uwbUpdated = false;
    if (uwb.timestamp != _lastUwbTimestamp) {
        _lastUwbTimestamp = uwb.timestamp;
        if (uwb.distCm >= 0.0f) {
            _fusedPose.uwbAngle  = uwb.angleDeg;
            _fusedPose.uwbDistCm = uwb.distCm;
            correct_bearing(uwb.angleDeg, imu.yaw, rtConfig.fusionRUwb);
            _distKalman.update(uwb.distCm, rtConfig.uwbKalmanQ, rtConfig.uwbKalmanR);
            _fusedPose.distanceCm = _distKalman.x;
            _fusedPose.timestamp  = now;
            uwbUpdated = true;
        }
    }

    // Update bearing from camera readings. The Kalman filter is applied in correct_bearing().
    bool camUpdated = false;
    if (cam.timestamp != _lastCameraTimestamp) {
        _lastCameraTimestamp = cam.timestamp;
        if (cam.found) {
            float camAngle = cam.posX * (CAMERA_H_FOV_DEG / 2.0f);
            _fusedPose.camAngle = camAngle;
            correct_bearing(camAngle, imu.yaw, rtConfig.fusionRCamera);
            _fusedPose.timestamp = now;
            camUpdated = true;
        }
    }

    // Convert absolute bearing back to relative angle using current compass heading.
    if (!isnan(_bearingDeg)) {
        float relAngle = imu.yaw - _bearingDeg;
        while (relAngle >  180.0f) relAngle -= 360.0f;
        while (relAngle < -180.0f) relAngle += 360.0f;
        _fusedPose.fusedAngle = relAngle;
    }

    // Update uncertainty and speed
    _fusedPose.uncertainty     = _bearingP + _innovEwma;
    _fusedPose.fusedSpeedMph   = rpm_get().speedMph;
    _fusedPose.fusedOdometryCm = _fusedOdometryCm;

    // Log updates to serial for debugging
    if (uwbUpdated || camUpdated) {
        const char* src = (uwbUpdated && camUpdated) ? "📡 uwb + 📷 blob"
                        : uwbUpdated                 ? "📡 uwb"
                                                     : "📷 blob";
        ESP_LOGD(TAG, "bearing=%.1f°  angle=%.1f°  uwbRaw=%.1f°  dist=%.0fcm  rawDist=%.0fcm  unc=%.1f  src=%s",
            _bearingDeg, _fusedPose.fusedAngle, _fusedPose.uwbAngle,
            _fusedPose.distanceCm, _fusedPose.uwbDistCm, _fusedPose.uncertainty, src);
    }
    return uwbUpdated || camUpdated;
}

const Pose& fusion_get() { return _fusedPose; }
