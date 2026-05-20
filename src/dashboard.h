#pragma once
#include <stdint.h>

struct PerfData {
    uint32_t imuAvg,  imuMax;
    uint32_t uwbAvg,  uwbMax;
    uint32_t navAvg,  navMax;
    uint32_t ctrlAvg, ctrlMax;
    uint32_t oledAvg, oledMax;
    uint32_t wifiAvg, wifiMax;
};

void dashboard_init();
void dashboard_set_perf(const PerfData& p);
void dashboard_update(float lps);
