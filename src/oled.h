#pragma once
#include "nav.h"
#include "control.h"
#include "imu.h"

void oled_init();
void oled_update(float lps, const NavData& nav, const ControlOutput& output, const ImuData& imu);
