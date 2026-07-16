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

## Benchmark #3 — 2026-07-13

**Note:** OLED decoupled from the loop — rendering + frame push moved to a dedicated
FreeRTOS task on core 0, and the panel moved to its own I2C controller (Wire1,
GPIO 17/18) so the ~26ms frame transfer no longer contends with sensor traffic on
the shared bus. Loop-side `oled_update()` is now just a snapshot+notify handoff.
Report format extended since #2: `lps` is a true window average (was a 100ms-slice
sample — earlier lps values are not directly comparable), plus new `maxLoop`
(longest gap between loop entries), `oledT` (core-0 render+push), `fus`, `rpm`,
`dash`, and `ser` columns. UWB is now the DW3000 AOA anchor on UART (no more
poll/stagger timing config).

**Loop config:**
```
perfImu.begin();    imu_update();                 perfImu.end();     // ACTIVE
perfUwb.begin();    uwb_update();                 perfUwb.end();     // ACTIVE
perfFusion.begin(); fusion_update();              perfFusion.end();  // ACTIVE
perfNav.begin();    nav_update();                 perfNav.end();     // ACTIVE
perfCtrl.begin();   control_update();             perfCtrl.end();    // ACTIVE
perfOled.begin();   oled_update(loopHz.hz);       perfOled.end();    // ACTIVE (snapshot+notify; render on core 0)
perfWifi.begin();   wifi_update();                perfWifi.end();    // ACTIVE
perfRpm.begin();    rpm_update();                 perfRpm.end();     // ACTIVE
perfDash.begin();   dashboard_update(loopHz.hz);  perfDash.end();    // ACTIVE
perfSerial.begin(); serial_hal_update();          perfSerial.end();  // ACTIVE
```

**Timing config:**
```
IMU_POLLING_INTERVAL_MS      = 10
IMU_REPORT_INTERVAL_MS       = 10
RPM_POLL_INTERVAL_MS         = 5
OLED_UPDATE_INTERVAL_MS      = 200
CONTROL_UPDATE_INTERVAL_MS   = 20
DASHBOARD_UPDATE_INTERVAL_MS = 100
DW3000_BAUD                  = 115200 (UART; ~50Hz ranging)
```

**UWB state:** DW3000 anchor live, tag present — ~50Hz ranging (sensor line reads
30–50Hz due to 100ms HzTracker slice quantization)

**Averaged results** _(computed from 13 perf report samples; first 2 skipped)_**:**

| Metric | Avg | Max (avg of maxes) |
|--------|-----|--------------------|
| Loop rate (lps) | 3971 | — |
| Max loop gap (µs) | — | 4,765 |
| IMU exec (µs) | 70 | 3,913 |
| UWB exec (µs) | 15 | 668 |
| Fusion exec (µs) | 6 | 65 |
| Nav exec (µs) | 2 | 30 |
| Ctrl exec (µs) | 59 | 195 |
| OLED handoff (µs) | 2 | 24 |
| OLED render task, core 0 (µs) | 26,539 | 26,842 |
| WiFi exec (µs) | 51 | 202 |
| RPM exec (µs) | 21 | 440 |
| Dashboard exec (µs) | 6 | 987 |
| Serial HAL exec (µs) | 3 | 28 |

**Sensor rates:** imu=100Hz, uwb=~50Hz, rpm=200Hz — all at target.

**Notes:** Worst-case loop stall dropped from ~48ms (shared-bus era: 25ms OLED frame
starving the Wire lock + ~20ms IMU event-backlog drain) to ~4.8ms, which is now just
the IMU's own active SHTP poll — `maxLoop ≈ imu max` confirms nothing else blocks.
The 50Hz control tick can no longer miss its window. OLED render still costs ~26.5ms
per frame but is paid entirely on core 0/Wire1 where nothing waits on it. Intermediate
experiment (chunked frame push with 1ms yields on the shared bus) made things worse —
fine-grained bus interleaving multiplied the BNO085's multi-transaction event reads to
~160ms stalls; dedicated bus was the correct fix. UWB coming online (0→50Hz) added no
measurable loop cost (fus/nav maxes up ~50µs).

---

## Benchmark #4 — 2026-07-16

**Note:** A/B test of a faster IMU SHTP drain gate (`IMU_POLLING_INTERVAL_MS` 10 → 2,
report interval unchanged at 10ms/100Hz), aiming to cut the ~5ms average queue-wait
on rotation events. **Result: reverted** — see notes. Loop no longer contains
fusion/nav (estimation moved Pi-side); baseline re-measured on this loop shape.

**Loop config:**
```
perfImu.begin();    imu_update();                 perfImu.end();     // ACTIVE
perfUwb.begin();    uwb_update();                 perfUwb.end();     // ACTIVE
perfCtrl.begin();   control_update();             perfCtrl.end();    // ACTIVE
pan_update(...);                                                     // ACTIVE
perfOled.begin();   oled_update(loopHz.hz);       perfOled.end();    // ACTIVE (render on core 0)
perfWifi.begin();   wifi_update();                perfWifi.end();    // ACTIVE
perfRpm.begin();    rpm_update();                 perfRpm.end();     // ACTIVE
perfDash.begin();   dashboard_update(loopHz.hz);  perfDash.end();    // ACTIVE
perfSerial.begin(); serial_hal_update();          perfSerial.end();  // ACTIVE
```

**Timing config:** as #3 except `IMU_POLLING_INTERVAL_MS` = 10 (A) vs 2 (B).

**UWB state:** anchor disconnected for both compared runs (uwb=0Hz); motor off,
dashboard closed. (Candidate run continued with the anchor plugged in mid-run —
those samples excluded from the averages.)

**Averaged results** _(A: 11 samples, B: 9 samples; first 2 skipped)_**:**

| Metric | A: 10ms gate | B: 2ms gate | Δ |
|--------|-------------|-------------|---|
| Loop rate (lps) | 4317 | 3273 | **−24%** |
| Max loop gap (µs) | 4,273 | 4,587 | +7% |
| IMU exec avg/max (µs) | 66 / 3,770 | 131 / 3,650 | avg +101% |
| RPM exec avg/max (µs) | 21 / 386 | 23 / 368 | avg +11% |
| Encoder rate (Hz) | 200 steady | 190–200 flicker | −2% avg |
| IMU report rate | 100Hz | 100Hz (unchanged) | — |

**Notes:** Failed the pass criteria (lps within 5%, enc steady 200Hz) and was
reverted. Empty SHTP polls are not cheap: each gate tick pays an I2C header
transaction whether or not events are pending, so 5× the poll rate doubled
imu avg exec and cost ~24% lps. The `[sensor perf] imu=…Hz` readout is a gate-tick
counter, not a report counter (`_imuHz.update()` runs every tick in imu_update), so
it read ~470Hz during run B — misleading; would need to move inside the
SH2_ROTATION_VECTOR case to measure real report rate. Side observation from run B's
UWB-connected tail: outlier-warning bursts coincide with uwb max ~9.2ms and
maxLoop 10–14ms spikes — serial log printing from the frame callback is a
measurable stall, worth its own look.

---

## Benchmark #5 — 2026-07-16

**Note:** Sensor-rate tuning series (runs A–F) following #4, after fixing `_imuHz` to
count rotation reports instead of gate ticks (so `imu=…Hz` is now trustworthy).
Landed config: **RV ~370 Hz + linear accel 50 Hz + encoder 250 Hz clean**, at
~3,080 lps (−32% from the 10ms-gate baseline — accepted trade).

**Loop config:** as #4. **Conditions:** UWB anchor disconnected, motor off, dashboard closed.

**Runs** _(averages; first 2 samples skipped)_**:**

| Run | IMU gate/report | Enc poll | lps | imu Hz | enc Hz | imu avg µs | Verdict |
|-----|----------------|----------|-----|--------|--------|-----------|---------|
| A | 10ms / 10ms | 5ms | 4499 | 100 | 200 | 64 | baseline |
| B | 2ms / 10ms | 5ms | 3390 | 100 | 196 | 128 | fail: −25% lps for 0 extra data |
| C | 5ms / 10ms | 5ms | 4101 | 100 | 200 | 82 | fail: −9% lps for 0 extra data |
| D | 5ms / 5ms | 5ms | ~3090 | 200 | 200 | ~150 | pass: 2× data, accepted lps cost |
| E | 5ms / 5ms | 3ms | ~2980 | ~196 | **220–320 jitter** | ~150 | fail: IMU drains (2–4ms) blow the 3ms window |
| F | 4ms / 4ms + LA 20ms | 4ms | ~3080 | **~370** | **250 steady** | ~146 | **pass — kept** |

**Notes:** Three durable lessons. (1) *Empty SHTP polls cost real loop time* — B/C
prove poll rate converts linearly to lps loss with zero data gain; always match the
drain gate to the report interval. (2) *Report rate is the data knob and the sensor
rounds it*: requesting 4ms delivered ~370 Hz rotation vectors (2.7ms grid), while 5ms
delivered exactly 200 — the BNO08x snaps to supported rates. (3) *Drain duration
bounds the encoder poll window*: each drain blocks ~1.5–4ms of I2C; run E's 3ms
encoder window couldn't coexist with 2-event drains. Fix that enabled F: linear accel
demoted to its own 20ms interval (only consumer is the 50 Hz telemetry `lax` field;
lay/laz unused), shortening drains enough for the 4ms window. Encoder velocity now
divides by the RateGate's measured dt instead of the nominal interval, so a delayed
poll can't read as inflated speed. Cogging `RPM_COGGING_CYCLE_SAMPLES` retuned 7→8
for the 4ms period; state-machine behavior at crawl speeds still needs a stand
re-validation. One-off `imu max` of 14.7ms observed in F (suspect: DCD auto-save
flash write — unconfirmed). Ceiling notes: BNO085 caps the shared bus at 400 kHz;
past this operating point the escape routes are dropping reports, the INT pin
(latency, not lps), or moving the BNO085 to SPI.

---
