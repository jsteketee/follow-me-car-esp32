// Navigation layer. Trilateration from left/right UWB anchor distances to compute tag heading and distance.
#include "nav.h"
#include "uwb.h"
#include "config.h"
#include "utils.h"
#include <math.h>
#include "esp_log.h"
#include "imu.h"

static const char *TAG = "nav";
static NavData _data = { NAN, NAN, 0.0f, NavState::STALE, 0 };
static HzTracker navHz;
static unsigned long lastProcessedTimestamp = 0;

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

static NavState navCheck(){
    if (millis() - _data.timestamp > UWB_STALE_HEADING_MS) return NavState::STALE;
    else return NavState::VALID;
}

void nav_update(const UWBReading& uwb, const ImuData& imu) {
    if (uwb.timestamp == lastProcessedTimestamp) return;

    float heading = calc_tag_heading(uwb);
    bool valid = !isnan(heading);
    navHz.update(valid);
    _data.updateHz = navHz.hz;

    if (valid) {
        _data.relativeAngle = heading;
        _data.distanceCm    = uwb.distFast;
        _data.timestamp     = millis();
    }
    _data.state = navCheck();
}

const NavData& nav_get() { return _data; }
