#pragma once
#include <stdint.h>

void oled_init();
void oled_update(float lps);
void oled_render_perf(uint32_t& avgUs, uint32_t& maxUs);  // core-0 render+push timing for the window (avg/max µs); resets on read
