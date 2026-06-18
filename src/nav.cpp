// Navigation layer.
// 1. Determine vehicle state
// 2. Update navigation mode and targets
#include "nav.h"
#include "fusion.h"
#include "config.h"
#include "runtime_config.h"
#include "utils.h"
#include "esp_log.h"
#include "imu.h"

static const char *TAG = "nav";
static NavData _navData = { 0.0f, 0.0f, DEFAULT_NAV_MODE, false };
static HzTracker _navHz;
static NavMode _prevNavMode = NavMode::STOPPED;

static const char* nav_mode_str(NavMode m) {
    switch (m) {
        case NavMode::FOLLOW_ME: return "FOLLOW_ME";
        case NavMode::TEST:      return "TEST";
        case NavMode::STOPPED:   return "STOPPED";
        default:                 return "UNKNOWN";
    }
}

void nav_set_mode(NavMode mode) {
    _navData.mode = mode;
}



void nav_update() {
    const Pose& fused = fusion_get();
    const ImuData&   imu   = imu_get();
    _navData.sensorsValid = fused.uncertainty < rtConfig.fusionStaleUncertainty;

    // Capture held heading on mode transitions or on first run when heading hold is uninitialized.
    if (_navData.mode != _prevNavMode) {
        ESP_LOGW(TAG, "🔀 nav mode: %s → %s", nav_mode_str(_prevNavMode), nav_mode_str(_navData.mode));
        if (_navData.mode != NavMode::FOLLOW_ME) _navData.headingHold = imu.yaw;
    }
    if (_navData.headingHold == 0.0f) _navData.headingHold = imu.yaw;
    _prevNavMode = _navData.mode;

    _navHz.update();
    _navData.updateHz = _navHz.hz;
}

const NavData& nav_get() { return _navData; }
