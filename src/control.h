#pragma once

struct ControlOutput {
    float throttle;        // -1.0 (full reverse) to 1.0 (full forward), 0.0 = neutral
    float steering;        // -1.0 (full left) to 1.0 (full right), 0.0 = center
    float targetSpeedMph;  // interpolated speed setpoint fed to PID this cycle; 0 when stopped
};

void control_init();
void control_update();
const ControlOutput& control_get();
