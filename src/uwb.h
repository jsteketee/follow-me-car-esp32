#pragma once

#include <Arduino.h>

struct UWBReading {
    float distLeft;
    float distRight;
    float distRear;
    unsigned long timestamp;
};

void uwb_init();
bool uwb_read(UWBReading &reading);
