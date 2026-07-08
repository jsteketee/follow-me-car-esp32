#pragma once

struct RPMData {
    float         rpm;
    float         speedMph;
    float         odometryCm;   // cumulative distance since startup
    unsigned long timestamp;
    bool          cogging;      // true when encoder detects motor oscillation without net forward motion
    int           signChanges;  // direction reversals in the current cogging analysis window
    float         encSpeedMph;  // AS5600 encoder EMA velocity (forward = positive); for Pi-side cogging detection
};

void           rpm_init();
bool           rpm_update();  // true when a new angle reading was successfully read from the encoder
const RPMData& rpm_get();
