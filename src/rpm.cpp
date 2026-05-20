// RPM sensor driver. Measures inter-pulse period via interrupt for per-pulse speed resolution.
// Speed zeroes out if no pulse arrives within RPM_STALE_MS (wheel stopped).
#include "rpm.h"
#include "config.h"
#include "utils.h"
#include "esp_log.h"
#include <Arduino.h>

static const char* TAG = "rpm";

static volatile uint32_t _lastPulseUs  = 0;
static volatile uint32_t _periodUs     = 0;
static volatile uint32_t _pulseCount   = 0;
static volatile bool     _pulseValid   = false;  // true once two pulses have been seen
static RPMData     _rpmData = {};
static KalmanFilter _speedKalman  = {};
static int          _spikeStreak  = 0;

static void IRAM_ATTR onPulse() {
    uint32_t now = micros();
    if (_pulseValid) {
        _periodUs = now - _lastPulseUs;
    }
    _lastPulseUs = now;
    _pulseValid  = true;
    _pulseCount++;
}

void rpm_init() {
    pinMode(PIN_RPM, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RPM), onPulse, FALLING);
    ESP_LOGI(TAG, "✅ RPM sensor ready on pin %d", PIN_RPM);
}

void rpm_update() {
    uint32_t lastPulseUs = _lastPulseUs;
    uint32_t periodUs    = _periodUs;
    uint32_t pulseCount  = _pulseCount;

    bool stale = (micros() - lastPulseUs) > (RPM_STALE_MS * 1000UL);

    if (stale || periodUs == 0) {
        // Reset filter so the first new pulse is trusted immediately, not ramped up from 0
        _speedKalman = {};
        _spikeStreak = 0;
        _rpmData.rpm      = 0;
        _rpmData.speedMph = 0;
    } else {
        float periodS  = periodUs / 1000000.0f;
        _rpmData.rpm      = (60.0f / periodS) / RPM_PULSES_PER_REV;
        float rawSpeed = _rpmData.rpm * RPM_SPEED_FACTOR;
        // Reject noise pulses that produce an implausibly large speed jump.
        // Force-accept after RPM_SPIKE_MAX_STREAK consecutive rejections so genuine acceleration isn't blocked.
        bool spike = (_speedKalman.initialized &&
                      rawSpeed > _rpmData.speedMph * RPM_SPIKE_REJECT_FACTOR &&
                      _spikeStreak < RPM_SPIKE_MAX_STREAK);
        if (spike) {
            ESP_LOGW(TAG, "⚠️ RPM spike rejected: raw=%.2f mph  filtered=%.2f mph  streak=%d",
                     rawSpeed, _rpmData.speedMph, _spikeStreak + 1);
            _speedKalman.p += RPM_KALMAN_Q;
            _spikeStreak++;
        } else {
            _rpmData.speedMph = _speedKalman.update(rawSpeed, RPM_KALMAN_Q, RPM_KALMAN_R);
            _spikeStreak   = 0;
        }
    }

    // Odometry: derive cm from the same factor (RPM_SPEED_FACTOR * 2682.24 = cm per motor rev)
    static uint32_t lastCount = 0;
    uint32_t pulses        = pulseCount - lastCount;
    lastCount              = pulseCount;
    _rpmData.odometryCm      += pulses * (RPM_SPEED_FACTOR * 2682.24f / RPM_PULSES_PER_REV);
    _rpmData.timestamp        = millis();

    static uint32_t lastLogMs = 0;
    if (millis() - lastLogMs >= 1000) {
        lastLogMs = millis();
        ESP_LOGV(TAG, "rpm=%.0f  speed=%.2fmph  odo=%.1fcm  period=%uus", _rpmData.rpm, _rpmData.speedMph, _rpmData.odometryCm, periodUs);
    }
}

const RPMData& rpm_get() {
    return _rpmData;
}
