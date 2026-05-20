#pragma once

struct RPMData {
    float         rpm;
    float         speedMph;
    float         odometryCm;   // cumulative distance since startup
    unsigned long timestamp;
};

void           rpm_init();
void           rpm_update();
const RPMData& rpm_get();
