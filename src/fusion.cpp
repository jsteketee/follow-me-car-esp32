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
static float         _layMean             = 0.0f;
static float         _layVariance         = 0.0f;

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

// Detects motor cogging: high-variance, near-zero-mean forward acceleration
// indicates the motor is oscillating without producing net forward movement.
static bool detect_cogging(float lay) {
    float dev     = lay - _layMean;
    _layMean     += COGGING_LAY_MEAN_ALPHA * dev;
    _layVariance += COGGING_LAY_VAR_ALPHA  * (dev * dev - _layVariance);
    bool raw      = (fabsf(_layMean) < COGGING_MEAN_THRESHOLD)
                 && (_layVariance    > COGGING_VAR_THRESHOLD);
    static bool     _cogging     = false;
    static uint32_t _lastTrueMs  = 0;
    if (raw) _lastTrueMs = millis();
    bool cogging = raw || (millis() - _lastTrueMs < COGGING_HOLD_MS);
    if (cogging != _cogging) {
        ESP_LOGI(TAG, "%s  mean=%.2f m/s²  var=%.2f (m/s²)²",
            cogging ? "🔴 cogging detected" : "🟢 cogging cleared",
            _layMean, _layVariance);
        _cogging = cogging;
    }
    return cogging;
}

void fusion_init() {
    ESP_LOGI(TAG, "✅ fusion ready  rUwb=%.1f  rCam=%.1f  stale=%.1f  innovMeanAlpha=%.3f  innovEwmaAlpha=%.3f",
        rtConfig.fusionRUwb, rtConfig.fusionRCamera, rtConfig.fusionStaleUncertainty,
        rtConfig.fusionInnovMeanAlpha, rtConfig.fusionInnovEwmaAlpha);
}

void fusion_update() {
    const ImuData&    imu = imu_get();
    const UWBReading& uwb = uwb_get();
    const CameraData& cam = camera_get();

    uint32_t now = millis();

    // Grow bearing uncertainty every loop as data becomes stale.
    if (_lastUpdateMs > 0) {
        float dt = (now - _lastUpdateMs) / 1000.0f;
        _bearingP += (rtConfig.fusionStaleUncertainty / rtConfig.sensorTimeoutSec) * dt;
    }
    _lastUpdateMs = now;

    // Perform Odom Dead reckoning gated by cogging detection. This updates distance in between UWB Readings. 
    bool cogging = detect_cogging(imu.lay);
    float rpmOdom = rpm_get().odometryCm;
    float traveled = rpmOdom - _lastOdometryCm;
    _lastOdometryCm = rpmOdom;
    if (!cogging) {
        _fusedOdometryCm += traveled;
        if (_distKalman.initialized)
            _distKalman.x = fmaxf(0.0f, _distKalman.x - traveled);
    }
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
    _fusedPose.fusedSpeedMph   = cogging ? 0.0f : rpm_get().speedMph;
    _fusedPose.fusedOdometryCm = _fusedOdometryCm;

    // Log updates to serial for debugging
    if (uwbUpdated || camUpdated) {
        const char* src = (uwbUpdated && camUpdated) ? "📡 uwb + 📷 blob"
                        : uwbUpdated                 ? "📡 uwb"
                                                     : "📷 blob";
        ESP_LOGI(TAG, "bearing=%.1f°  angle=%.1f°  uwbRaw=%.1f°  dist=%.0fcm  rawDist=%.0fcm  unc=%.1f  src=%s",
            _bearingDeg, _fusedPose.fusedAngle, _fusedPose.uwbAngle,
            _fusedPose.distanceCm, _fusedPose.uwbDistCm, _fusedPose.uncertainty, src);
    }
}

const Pose& fusion_get() { return _fusedPose; }
