#pragma once
#include <stdint.h>

// serial_hal — 50 Hz USB-CDC JSON telemetry stream for the Pi ROS2 bridge, plus the
// non-blocking RX path for command frames (target_speed + target_heading).

// Latest validated command frames from the Pi, one slot per frame shape. Each timestamp
// is stamped only when a frame of that shape parses AND validates; 0 means no valid
// frame of that shape has ever arrived. Both shapes are parsed regardless of the current
// control mode (the dashboard is the mode authority; control decides what to act on).
struct CommandData {
    // Setpoint frames (REMOTE): {"target_speed":..,"target_heading":..}
    float    targetSpeedMph;    // validated target speed (mph, [0, rtConfig.maxSpeedMph])
    float    targetHeadingDeg;  // validated absolute compass heading, normalized to [0, 360)
    uint32_t lastCmdMs;         // millis() of the last accepted setpoint frame; feeds the REMOTE cmd-timeout failsafe
    // Raw-actuator frames (DIRECT): {"throttle":..,"steering":..}
    float    directThrottle;    // validated normalized throttle [0, 1] (no reverse for now)
    float    directSteering;    // validated normalized steering [-1, 1]
    uint32_t lastDirectMs;      // millis() of the last accepted direct frame; feeds the DIRECT cmd-timeout failsafe
    // Optional in any frame shape: {"target_pan":..}
    float    targetPanDeg;      // validated UWB pan target (deg, 0 = car nose, +right); holds its last value when absent — and on cmd timeout (pan deliberately has no failsafe action)
};

void               serial_hal_init();
void               serial_hal_update();
const CommandData& serial_hal_get();
