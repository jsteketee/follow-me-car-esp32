// Sensor fusion layer.
// Tracks the tag's absolute compass bearing, blending UWB trilateration and camera blob
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

static Pose     _fusedPose          = { .fusedAngle = 0.0f, .uwbAngle = NAN, .camAngle = NAN, .distanceCm = -1.0f };
static float         _bearingDeg          = NAN;     // absolute compass bearing of tag (0–360°)
static float         _bearingP            = 1000.0f; // bearing estimate variance (deg²)
static float         _innovEwma           = 0.0f;    // EWMA of innov² (deg²) — spikes on surprise, decays on consistent readings
static uint32_t      _lastUpdateMs        = 0;
static unsigned long _lastUwbTimestamp    = 0;
static unsigned long _lastCameraTimestamp = 0;
static float         _lastOdometryCm      = 0.0f;
static float         _fusedOdometryCm     = 0.0f;
static float         _layMean             = 0.0f;
static float         _layVariance         = 0.0f;

// Returns angle to tag in degrees relative to car's forward axis.
// 0° = straight ahead, positive = right, negative = left. NAN if data invalid.
static float calc_tag_heading(const UWBReading& uwb) {
    if (uwb.distLeft < 0 || uwb.distRight < 0) {
        // ESP_LOGW(TAG, "⚠️ invalid UWB reading (dL=%.0f dR=%.0f)", uwb.distLeft, uwb.distRight);
        return NAN;
    }

    float dL = uwb.distLeft;
    float dR = uwb.distRight;
    float d  = UWB_ANCHOR_SEPARATION_CM;

    float x        = (dL * dL - dR * dR) / (2.0f * d);
    float ySquared = dL * dL - (x + d / 2.0f) * (x + d / 2.0f);

    if (ySquared < 0) {
        ESP_LOGW(TAG, "⚠️ geometry failure (dL:%.0f dR:%.0f x:%.0f ySquared:%.1f)", dL, dR, x, ySquared);
        return NAN;
    }

    float y = sqrtf(ySquared);

    // Front/back disambiguation latch. Updated when distFront is valid; held on dropout.
    // Abrupt flips (heading change > UWB_FRONT_FLIP_ABRUPT_DEG) require UWB_FRONT_FLIP_CONFIRM
    // consecutive readings before the latch flips. Any agreeing reading resets the streak.
    static bool _tagIsBehind = false;
    static int  _flipStreak  = 0;
    if (uwb.distFront > 0) {
        float dx            = x - UWB_FRONT_X_CM;
        float distIfForward = sqrtf(dx*dx + (y - UWB_FRONT_Y_CM)*(y - UWB_FRONT_Y_CM));
        float distIfBehind  = sqrtf(dx*dx + (y + UWB_FRONT_Y_CM)*(y + UWB_FRONT_Y_CM));
        bool behind = fabsf(uwb.distFront - distIfBehind) < fabsf(uwb.distFront - distIfForward);
        if (behind != _tagIsBehind) {
            float h1 = atan2f(x,  y) * 180.0f / M_PI;
            float h2 = atan2f(x, -y) * 180.0f / M_PI;
            float delta = fabsf(h1 - h2);
            if (delta > 180.0f) delta = 360.0f - delta;
            bool abrupt = delta > UWB_FRONT_FLIP_ABRUPT_DEG;
            if (!abrupt || ++_flipStreak >= UWB_FRONT_FLIP_CONFIRM) {
                ESP_LOGI(TAG, "↕ tag side: %s → %s  (distFront=%.0f fwd_exp=%.0f beh_exp=%.0f  delta=%.0f° streak=%d)",
                         _tagIsBehind ? "behind" : "front", behind ? "behind" : "front",
                         uwb.distFront, distIfForward, distIfBehind, delta, _flipStreak);
                _tagIsBehind = behind;
                _flipStreak  = 0;
            } else {
                ESP_LOGD(TAG, "⟳ side flip deferred: delta=%.0f° streak=%d/%d  (distFront=%.0f)",
                         delta, _flipStreak, UWB_FRONT_FLIP_CONFIRM, uwb.distFront);
            }
        } else {
            _flipStreak = 0;
        }
    }
    if (_tagIsBehind) y = -y;

    float heading = atan2f(x, y) * 180.0f / M_PI;
    // ESP_LOGI(TAG, "✅ heading=%.1f°  dL=%.0f dR=%.0f  side=%s", heading, dL, dR, _tagIsBehind ? "behind" : "front");
    return heading;
}

// Update the absolute bearing estimate from a new relative angle measurement.
// r = measurement noise variance (deg²). Lower r = more trust in this measurement.
// imu.yaw increases counterclockwise; our relAngle convention is positive = right (clockwise),
// so absolute bearing = imuYaw - relAngle.
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
    ESP_LOGI(TAG, "✅ fusion ready");
}

void fusion_update() {
    const ImuData&    imu = imu_get();
    const UWBReading& uwb = uwb_get();
    const CameraData& cam = camera_get();

    uint32_t now = millis();

    // Seed bearing from IMU yaw once it has a valid reading, so steering is active before the first sensor fix.
    if (isnan(_bearingDeg) && imu.update_hz > 0) {
        _bearingDeg = imu.yaw;
        ESP_LOGI(TAG, "bearing seeded from IMU: %.1f°", _bearingDeg);
    }

    // Grow bearing uncertainty every loop — covers both between-fix gaps and sensor dropout.
    if (_lastUpdateMs > 0) {
        float dt = (now - _lastUpdateMs) / 1000.0f;
        _bearingP += (rtConfig.fusionStaleUncertainty / rtConfig.sensorTimeoutSec) * dt;
    }
    _lastUpdateMs = now;

    // Cogging detection runs first so the result is available for dead reckoning below.
    bool cogging = detect_cogging(imu.lay);

    // Dead reckoning: subtract distance traveled since last loop from distanceCm.
    // UWB overwrites with a fresh value when a reading arrives, so this only matters during dropout.
    // Skip odometry increment during cogging — false RPM pulses would otherwise shrink distanceCm.
    float rpmOdom = rpm_get().odometryCm;
    float traveled = rpmOdom - _lastOdometryCm;
    _lastOdometryCm = rpmOdom;
    if (!cogging) {
        _fusedOdometryCm += traveled;
        if (_fusedPose.distanceCm >= 0.0f)
            _fusedPose.distanceCm = fmaxf(0.0f, _fusedPose.distanceCm - traveled);
    }

    bool uwbUpdated = false;
    if (uwb.timestamp != _lastUwbTimestamp) {
        _lastUwbTimestamp = uwb.timestamp;
        float heading = calc_tag_heading(uwb);
        if (!isnan(heading)) {
            _fusedPose.uwbAngle = heading;
            correct_bearing(heading, imu.yaw, rtConfig.fusionRUwb);
            _fusedPose.distanceCm = uwb.distFast;
            _fusedPose.timestamp  = now;
            uwbUpdated = true;
        }
    }

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
    // Negated because imu.yaw increases counterclockwise, but our angle convention is positive = right.
    if (!isnan(_bearingDeg)) {
        float relAngle = imu.yaw - _bearingDeg;
        while (relAngle >  180.0f) relAngle -= 360.0f;
        while (relAngle < -180.0f) relAngle += 360.0f;
        _fusedPose.fusedAngle = relAngle;
    }

    _fusedPose.uncertainty     = _bearingP + _innovEwma;
    _fusedPose.fusedSpeedMph   = cogging ? 0.0f : rpm_get().speedMph;
    _fusedPose.fusedOdometryCm = _fusedOdometryCm;

    if (uwbUpdated || camUpdated) {
        ESP_LOGI(TAG, "bearing=%.1f°  angle=%.1f°  dist=%.0fcm  unc=%.1f  src=%s",
            _bearingDeg, _fusedPose.fusedAngle, _fusedPose.distanceCm,
            _fusedPose.uncertainty,
            camUpdated ? "📷 blob" : "📡 uwb");
    }
}

const Pose& fusion_get() { return _fusedPose; }
