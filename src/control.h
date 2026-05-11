#pragma once

#include "uwb.h"

struct ControlOutput {
    float throttle;
    float steering;
};

void control_init();
ControlOutput control_update(const UWBReading &reading, float heading, float speed);
