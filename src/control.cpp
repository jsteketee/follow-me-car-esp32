#include "control.h"
#include "config.h"

void control_init() {
    // TODO: initialize PWM outputs
}

ControlOutput control_update(const UWBReading &reading, float heading, float speed) {
    // TODO: trilateration, path following, pace coaching
    return {0.0f, 0.0f};
}
