// Navigation layer.
// 1. Consume and process sensor data
// 2. Determine vehicle state
// 3. Update navigation mode and targets
#include "nav.h"
#include "uwb.h"
#include "config.h"
#include "utils.h"
#include <math.h>
#include "esp_log.h"
#include "imu.h"

static const char *TAG = "nav";
static NavData _navData = { NAN, NAN, 0.0f, 0.0f, NavState::STOPPED, 0 };
static HzTracker _navHz;
static unsigned long lastProcessedTimestamp = 0;
static NavState _prevState = NavState::STOPPED;

// Returns angle to tag in degrees relative to car's forward axis.
// 0° = straight ahead, positive = right, negative = left. NAN if data invalid.
static float calc_tag_heading(const UWBReading& uwb) {
    bool newReading = uwb.timestamp != lastProcessedTimestamp;

    if (!uwb.validLeft || !uwb.validRight) {
        if (newReading) {
            lastProcessedTimestamp = uwb.timestamp;
            ESP_LOGW(TAG, "⚠️ invalid UWB reading (validLeft=%d validRight=%d)", uwb.validLeft, uwb.validRight);
        }
        return NAN;
    }

    float dL = uwb.distLeft;
    float dR = uwb.distRight;
    float d  = UWB_ANCHOR_SEPARATION_CM;

    float x        = (dL * dL - dR * dR) / (2.0f * d);
    float ySquared = dL * dL - (x + d / 2.0f) * (x + d / 2.0f);

    if (ySquared < 0) {
        if (newReading) {
            lastProcessedTimestamp = uwb.timestamp;
            ESP_LOGW(TAG, "⚠️ geometry failure (dL:%.0f dR:%.0f x:%.0f ySquared:%.1f)", dL, dR, x, ySquared);
        }
        return NAN;
    }

    lastProcessedTimestamp = uwb.timestamp;
    float y       = sqrtf(ySquared);
    float heading = atan2f(x, y) * 180.0f / M_PI;
    if (newReading){
        ESP_LOGI(TAG, "✅ heading=%.1f°  dL=%.0f dR=%.0f", heading, dL, dR);
    }
    return heading;
}

void nav_set_mode(NavState mode) {
    _navData.state = mode;
}

boolean nav_is_stale() {
    boolean stale = false;
    if (_navData.state == NavState::FOLLOW_ME){
        if (millis() - _navData.timestamp > UWB_STALE_HEADING_MS) {
            stale = true;
        }
        if (_navData.relativeAngle == NAN || _navData.distanceCm == NAN){
            stale = true;
        }
    }
    return stale;
}

void nav_update() {
    const UWBReading& uwb = uwb_get();
    const ImuData&    imu = imu_get();

    
    // Stale check runs every call so NORMAL→STALE fires even if UWB stops producing new readings
    if (nav_is_stale()) _navData.state = NavState::STALE;

    // Capture held heading on mode transitions; late-latch on first update
    if (_navData.state != _prevState) {
        if (_navData.state != NavState::FOLLOW_ME) _navData.headingHold = imu.yaw;
    }
    if (_navData.headingHold == 0.0f) _navData.headingHold = imu.yaw;
    _prevState = _navData.state;

    if (uwb.timestamp == lastProcessedTimestamp) return;

    float heading = calc_tag_heading(uwb);
    bool valid = !isnan(heading);
    _navHz.update(valid);
    _navData.updateHz = _navHz.hz;

    if (valid) {
        _navData.relativeAngle = heading;
        _navData.distanceCm    = uwb.distFast;
        _navData.timestamp     = millis();
    }
}

const NavData& nav_get() { return _navData; }
