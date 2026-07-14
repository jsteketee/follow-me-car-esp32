#pragma once

// Pan servo driver — aims the UWB anchor. Owns the µs↔degree map (calibrated via
// the pan-cal env); reports angles in the anchor frame: 0 = boresight forward,
// positive = right, matching the UWB bearing convention.

void  pan_init();                    // attach the servo and move to trimmed center
void  pan_update(float targetDeg);   // call each loop with the CommandData pan target: 20ms-gated slew toward it
float pan_get_angle();               // current commanded pan angle in degrees (slew-limited, so honest during moves)

#ifdef PAN_CAL_TEST
// Blocking bench calibration (~40s): mechanical sweep, then a five-point UWB-referenced
// least-squares fit of the µs↔degree map. Requires uwb_init() done, a stationary tag
// roughly ahead of the car, and prints suggested config values when finished.
void pan_calibrate();
#endif
