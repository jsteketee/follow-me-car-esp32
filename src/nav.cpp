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
static NavData _navData = { 0.0f, -1.0f, 0.0f, 0.0f, DEFAULT_NAV_MODE, 0 };
static HzTracker _navHz;
static unsigned long lastProcessedTimestamp = 0;
static NavMode _prevState = NavMode::STOPPED;

// Returns angle to tag in degrees relative to car's forward axis.
// 0° = straight ahead, positive = right, negative = left. NAN if data invalid.
static float calc_tag_heading(const UWBReading& uwb) {
    bool newReading = uwb.timestamp != lastProcessedTimestamp;

    if (uwb.distLeft < 0 || uwb.distRight < 0) {
        if (newReading) {
            lastProcessedTimestamp = uwb.timestamp;
            ESP_LOGW(TAG, "⚠️ invalid UWB reading (dL=%.0f dR=%.0f)", uwb.distLeft, uwb.distRight);
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
    float y = sqrtf(ySquared);

    // Front/back disambiguation latch. Updated when distFront is valid; held on dropout.
    // Abrupt flips (heading change > UWB_FRONT_FLIP_ABRUPT_DEG) require UWB_FRONT_FLIP_CONFIRM
    // consecutive readings before the latch flips. Any agreeing reading resets the streak.
    static bool tagIsBehind = false;
    static int  flipStreak  = 0;
    if (uwb.distFront > 0) {
        float dx            = x - UWB_FRONT_X_CM;
        float distIfForward = sqrtf(dx*dx + (y - UWB_FRONT_Y_CM)*(y - UWB_FRONT_Y_CM));
        float distIfBehind  = sqrtf(dx*dx + (y + UWB_FRONT_Y_CM)*(y + UWB_FRONT_Y_CM));
        bool behind = fabsf(uwb.distFront - distIfBehind) < fabsf(uwb.distFront - distIfForward);
        if (behind != tagIsBehind) {
            float h1 = atan2f(x,  y) * 180.0f / M_PI;
            float h2 = atan2f(x, -y) * 180.0f / M_PI;
            float delta = fabsf(h1 - h2);
            if (delta > 180.0f) delta = 360.0f - delta;
            bool abrupt = delta > UWB_FRONT_FLIP_ABRUPT_DEG;
            if (!abrupt || ++flipStreak >= UWB_FRONT_FLIP_CONFIRM) {
                ESP_LOGI(TAG, "↕ tag side: %s → %s  (distFront=%.0f fwd_exp=%.0f beh_exp=%.0f  delta=%.0f° streak=%d)",
                         tagIsBehind ? "behind" : "front", behind ? "behind" : "front",
                         uwb.distFront, distIfForward, distIfBehind, delta, flipStreak);
                tagIsBehind = behind;
                flipStreak  = 0;
            } else {
                ESP_LOGD(TAG, "⟳ side flip deferred: delta=%.0f° streak=%d/%d  (distFront=%.0f)",
                         delta, flipStreak, UWB_FRONT_FLIP_CONFIRM, uwb.distFront);
            }
        } else {
            flipStreak = 0;
        }
    }
    if (tagIsBehind) y = -y;

    float heading = atan2f(x, y) * 180.0f / M_PI;
    if (newReading) {
        ESP_LOGI(TAG, "✅ heading=%.1f°  dL=%.0f dR=%.0f  side=%s", heading, dL, dR, tagIsBehind ? "behind" : "front");
    }
    return heading;
}

static const char* nav_mode_str(NavMode m) {
    switch (m) {
        case NavMode::FOLLOW_ME: return "FOLLOW_ME";
        case NavMode::STALE:     return "STALE";
        case NavMode::TEST:      return "TEST";
        case NavMode::STOPPED:   return "STOPPED";
        default:                 return "UNKNOWN";
    }
}

void nav_set_mode(NavMode mode) {
    _navData.mode = mode;
}



void nav_update() {
    const UWBReading& uwb = uwb_get();
    const ImuData&    imu = imu_get();

    // Guard on _navData.timestamp > 0 so startup doesn't immediately trip STALE before the first heading arrives.
    if ((_navData.mode == NavMode::FOLLOW_ME || _navData.mode == NavMode::STALE)) {
        bool stale = millis() - _navData.timestamp > FIX_TIMEOUT_MS;
        if      (stale  && _navData.mode == NavMode::FOLLOW_ME) _navData.mode = NavMode::STALE;
        else if (!stale && _navData.mode == NavMode::STALE)     _navData.mode = NavMode::FOLLOW_ME;
    }

    // Capture held heading on mode transitions or on first run when heading hold is uninitialized.
    if (_navData.mode != _prevState) {
        ESP_LOGW(TAG, "🔀 nav mode: %s → %s", nav_mode_str(_prevState), nav_mode_str(_navData.mode));
        if (_navData.mode != NavMode::FOLLOW_ME) _navData.headingHold = imu.yaw;
    }
    if (_navData.headingHold == 0.0f) _navData.headingHold = imu.yaw;
    _prevState = _navData.mode;

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
