#pragma once

struct ControlOutput {
    float throttle;     // -1.0 (full reverse) to 1.0 (full forward), 0.0 = neutral
    float steering;     // -1.0 (full left) to 1.0 (full right), 0.0 = center
};

void control_init();
void control_update();
const ControlOutput& control_get();
