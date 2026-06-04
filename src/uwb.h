#pragma once

#include <Arduino.h>

struct UWBReading {
    float distLeft;
    float distRight;
    float distFront;
    float distFast; //raw distance, no kallman filtering
    unsigned long timestamp;
};

void            uwb_init();
void            uwb_update();
const UWBReading& uwb_get();
void            uwb_run_diagnostics();
