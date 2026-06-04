# Performance Benchmarks

Each entry includes averaged results computed from all steady-state log samples, the active loop configuration, timing config, UWB state, and a short note.

---

## How to log a benchmark

1. Paste the full serial log output into the chat
2. Specify what changed since the last run
3. Claude will:
   - **Skip the first 2-3 samples** after boot (still stabilizing)
   - **Average all remaining steady-state samples** for lps, imu/uwb/nav/ctrl/oled/wifi avg+max, and rotation interval avg/max/jitter
   - Format the scorecard in chat and append an entry to this file

---

## Template

**Date:**
**Note:** _(what changed or what is being tested)_

**Loop config:**
```
perfImu.begin();  imu_update();      perfImu.end();   // ACTIVE or // DISABLED
perfUwb.begin();  uwb_update();      perfUwb.end();   // ACTIVE or // DISABLED
perfNav.begin();  nav_update();      perfNav.end();   // ACTIVE or // DISABLED
perfCtrl.begin(); control_update();  perfCtrl.end();  // ACTIVE or // DISABLED
perfOled.begin(); oled_update();     perfOled.end();  // ACTIVE or // DISABLED
perfWifi.begin(); wifi_update();     perfWifi.end();  // ACTIVE or // DISABLED
rpm_update();                                         // ACTIVE or // DISABLED
dashboard_update(loopHz.hz);                          // ACTIVE or // DISABLED
```

**Timing config:**
```
IMU_POLLING_INTERVAL_MS    = X
IMU_REPORT_INTERVAL_MS     = X
UWB_POLL_INTERVAL_MS       = X
UWB_ANCHOR_STAGGER_MS      = X
UWB_RESPONSE_TIMEOUT_MS    = X
OLED_UPDATE_INTERVAL_MS    = X
CONTROL_UPDATE_INTERVAL_MS = X
```

**UWB state:** _(e.g. all modules timing out / left+right responding / tag in range)_

**Averaged results** _(computed from all steady-state samples, first 2-3 skipped)_**:**

| Metric | Avg | Max (avg of maxes) |
|--------|-----|--------------------|
| Loop rate (lps) | X | — |
| IMU exec (µs) | X | X |
| UWB exec (µs) | X | X |
| Nav exec (µs) | X | X |
| Ctrl exec (µs) | X | X |
| OLED exec (µs) | X | X |
| WiFi exec (µs) | X | X |

| IMU rotation interval | Value |
|-----------------------|-------|
| avg | Xms |
| min (avg of mins) | Xms |
| max (avg of maxes) | Xms |
| jitter (avg of jitters) | Xms |

**Notes:**

---

## Benchmark #1 — 2026-05-20

**Note:** Baseline — all modules enabled, OLED on, UWB timing out (no tag present)

**Loop config:**
```
perfImu.begin();  imu_update();               perfImu.end();   // ACTIVE
perfUwb.begin();  uwb_update();               perfUwb.end();   // ACTIVE
perfNav.begin();  nav_update();               perfNav.end();   // ACTIVE
perfCtrl.begin(); control_update();           perfCtrl.end();  // ACTIVE
perfOled.begin(); oled_update(loopHz.hz);     perfOled.end();  // ACTIVE
perfWifi.begin(); wifi_update();              perfWifi.end();  // ACTIVE
rpm_update();                                                  // ACTIVE
dashboard_update(loopHz.hz);                                   // ACTIVE
// uwb_passthrough_update();                                   // DISABLED
```

**Timing config:**
```
IMU_POLLING_INTERVAL_MS    = 10
IMU_REPORT_INTERVAL_MS     = 10
UWB_POLL_INTERVAL_MS       = 50
UWB_ANCHOR_STAGGER_MS      = 25
UWB_RESPONSE_TIMEOUT_MS    = 500
OLED_UPDATE_INTERVAL_MS    = 100
CONTROL_UPDATE_INTERVAL_MS = 20
```

**UWB state:** All modules timing out — no tag present

**Averaged results** _(33 rotation interval samples, 34 perf report samples; first 2 skipped)_**:**

| Metric | Avg | Max (avg of maxes) |
|--------|-----|--------------------|
| Loop rate (lps) | 2760 | — |
| IMU exec (µs) | 142 | 25,000 |
| UWB exec (µs) | 8 | 8,890 |
| Nav exec (µs) | 4 | 9,200 |
| Ctrl exec (µs) | 52 | 145 |
| OLED exec (µs) | 93 | 25,100 |
| WiFi exec (µs) | 42 | 145 |

| IMU rotation interval | Value |
|-----------------------|-------|
| avg | 12.7ms |
| min (avg of mins) | 2.8ms |
| max (avg of maxes) | 41ms |
| jitter (avg of jitters) | 38ms |

**Notes:** OLED 25ms I2C transfer every 100ms is the primary bottleneck — causes I2C bus contention with IMU, depressed loop rate (~2760 lps), and high IMU jitter (~38ms avg).

---

## Benchmark #2 — 2026-05-20

**Note:** UWB live ranging — left+right anchors responding, tag at ~200cm (~8–25° right of center)

**Loop config:**
```
perfImu.begin();  imu_update();               perfImu.end();   // ACTIVE
perfUwb.begin();  uwb_update();               perfUwb.end();   // ACTIVE
perfNav.begin();  nav_update();               perfNav.end();   // ACTIVE
perfCtrl.begin(); control_update();           perfCtrl.end();  // ACTIVE
perfOled.begin(); oled_update(loopHz.hz);     perfOled.end();  // ACTIVE
perfWifi.begin(); wifi_update();              perfWifi.end();  // ACTIVE
rpm_update();                                                  // ACTIVE
dashboard_update(loopHz.hz);                                   // ACTIVE
```

**Timing config:**
```
IMU_POLLING_INTERVAL_MS    = 10
IMU_REPORT_INTERVAL_MS     = 10
UWB_POLL_INTERVAL_MS       = 50
UWB_ANCHOR_STAGGER_MS      = 25
UWB_RESPONSE_TIMEOUT_MS    = 500
OLED_UPDATE_INTERVAL_MS    = 100
CONTROL_UPDATE_INTERVAL_MS = 20
```

**UWB state:** Left + right anchors responding; tag at ~200cm, 8–25° right of center (~5Hz ranging rate)

**Averaged results** _(computed from 8 perf report samples, 10 rotation interval samples; first 2 skipped)_**:**

| Metric | Avg | Max (avg of maxes) |
|--------|-----|--------------------|
| Loop rate (lps) | 2425 | — |
| IMU exec (µs) | 161 | 24,000 |
| UWB exec (µs) | 5 | 599 |
| Nav exec (µs) | 16 | 7,800 |
| Ctrl exec (µs) | 53 | 143 |
| OLED exec (µs) | 104 | 25,400 |
| WiFi exec (µs) | 42 | 152 |

| IMU rotation interval | Value |
|-----------------------|-------|
| avg | 12.4ms |
| min (avg of mins) | 2.8ms |
| max (avg of maxes) | 38.3ms |
| jitter (avg of jitters) | 35.6ms |

**Notes:** UWB exec avg drops from 8µs (timeout path) to 5µs when anchors are responding — timeout path is slightly more expensive due to repeated millis() checks. OLED remains the primary bottleneck (25ms I2C transfer). UWB ranging cycle is ~200ms (50ms poll interval + sequential left→right polling each taking up to ~100ms). IMU jitter is comparable to Benchmark #1 — OLED I2C contention still dominant.

---
