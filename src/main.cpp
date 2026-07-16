// Wiring layer. Calls _update() on each module, fetches their output via _get(), and routes data to consumers as parameters.
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "utils.h"
#include "oled.h"
#include "imu.h"
#include "uwb.h"
#include "control.h"
#include "wifi_config.h"
#include "rpm.h"
#include "dashboard.h"
#include "serial_hal.h"
#include "pan.h"
#include "esp_log.h"

static const char* TAG = "perf";
static HzTracker   loopHz;

// Execution time per module (avg/max µs over the reporting window).
static PerfTracker perfImu, perfUwb,
                   perfCtrl, perfOled, perfWifi, perfRpm, perfDash, perfSerial;

// New-data arrival rate for actual sensors only (not control, etc.) —
// counts updates that delivered fresh data, not polls that came back empty.
// IMU poll rate is exposed via imu_get().update_hz (tracked internally by imu_update).
static HzTracker hzUwb, hzEnc;

static uint32_t lastReport   = 0;
static uint32_t maxLoopGapUs = 0;  // longest gap between loop() entries in the window — worst-case latency between control opportunities

// Resets every perf tracker and the loop-gap max, and stamps the window start.
static void perf_window_reset() {
    perfImu.reset();    perfUwb.reset();
    perfCtrl.reset();   perfOled.reset(); perfWifi.reset();
    perfRpm.reset();    perfDash.reset(); perfSerial.reset();
    maxLoopGapUs = 0;
    lastReport   = millis();
}

// Prints the once-per-second perf heartbeat and feeds the dashboard perf panel.
static void perf_report() {
    if (millis() - lastReport < 1000) return;

    // First window: discard samples accumulated during/just after setup so every
    // reported window is a clean, full second.
    if (lastReport == 0) {
        uint32_t discardAvg, discardMax;
        oled_render_perf(discardAvg, discardMax);
        perf_window_reset();
        return;
    }

    const ImuData& imu = imu_get();

    // True window-average loop rate: every tracker counts once per iteration.
    uint32_t windowMs = millis() - lastReport;
    float lps = perfImu.count * 1000.0f / windowMs;

    // Core-0 render task timing, tracked separately from the loop-side perfOled
    // (which only measures the snapshot+notify handoff).
    uint32_t oledTaskAvg, oledTaskMax;
    oled_render_perf(oledTaskAvg, oledTaskMax);

    Serial.printf(
        "[loop perf] lps=%.0f"
        "  maxLoop=%u"
        "  imu=%u/%u"
        "  uwb=%u/%u"
        "  ctrl=%u/%u"
        "  oled=%u/%u"
        "  oledT=%u/%u"
        "  wifi=%u/%u"
        "  rpm=%u/%u"
        "  dash=%u/%u"
        "  ser=%u/%u"
        "  (avg/max us)\n",
        lps,
        maxLoopGapUs,
        perfImu.avg(),    perfImu.maxUs,
        perfUwb.avg(),    perfUwb.maxUs,
        perfCtrl.avg(),   perfCtrl.maxUs,
        perfOled.avg(),   perfOled.maxUs,
        oledTaskAvg,      oledTaskMax,
        perfWifi.avg(),   perfWifi.maxUs,
        perfRpm.avg(),    perfRpm.maxUs,
        perfDash.avg(),   perfDash.maxUs,
        perfSerial.avg(), perfSerial.maxUs);

    Serial.printf(
        "[sensor perf] imu=%.0fHz  uwb=%.0fHz  enc=%.0fHz  hall=%.0fHz\n",
        (float)imu.update_hz, hzUwb.hz, hzEnc.hz, rpm_get().hallHz);

    dashboard_set_perf({ perfImu.avg(),    perfImu.maxUs,
                         perfUwb.avg(),    perfUwb.maxUs,
                         perfCtrl.avg(),   perfCtrl.maxUs,
                         oledTaskAvg,      oledTaskMax,
                         perfWifi.avg(),   perfWifi.maxUs });

    perf_window_reset();
}

void setup()
{
    delay(3000);
    Serial.begin(115200);
    Serial.println("⭐⭐⭐⭐⭐ Setting Up ⭐⭐⭐⭐⭐");

    //Init each device
    wifi_init();
    oled_init();
    // Sensor I2C bus (BNO085 + AS5600). The OLED runs on its own controller
    // (Wire1, opened in oled_init) so its frame pushes never contend with sensors.
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    imu_init();
    uwb_init();
    control_init();
    pan_init();
    rpm_init();
    dashboard_init();
    serial_hal_init();

#ifdef PAN_CAL_TEST
    // Blocking ~20s bench calibration (pan-cal env) — needs uwb_init() done and a
    // stationary tag roughly dead ahead. Prints suggested config values when done.
    pan_calibrate();
#endif

    control_set_mode(DEFAULT_CONTROL_MODE);
    Serial.println("⭐⭐⭐⭐⭐ Setup Complete ⭐⭐⭐⭐⭐");
}

void loop()
{
    loopHz.update();

    // Longest gap between loop entries this window — includes the iteration's own
    // execution time, so it directly bounds how late any module's gate can fire.
    static uint32_t _lastLoopUs = 0;
    uint32_t nowUs = micros();
    if (_lastLoopUs != 0 && nowUs - _lastLoopUs > maxLoopGapUs) maxLoopGapUs = nowUs - _lastLoopUs;
    _lastLoopUs = nowUs;

    perfImu.begin();    imu_update();                              perfImu.end();
    perfUwb.begin();    hzUwb.update(uwb_update());               perfUwb.end();
    perfCtrl.begin();   bool ctrlTicked = control_update();       perfCtrl.end();
    if (ctrlTicked) dashboard_sample_ctrl();  // buffer a 50 Hz graph sample per PID tick
    pan_update(serial_hal_get().targetPanDeg);  // µs-cheap: 20ms-gated slew toward the Pi-commanded pan target
    perfOled.begin();   oled_update(loopHz.hz);                   perfOled.end();
    perfWifi.begin();   wifi_update();                            perfWifi.end();
    perfRpm.begin();    hzEnc.update(rpm_update(imu_get().lax));  perfRpm.end();
    perfDash.begin();   dashboard_update(loopHz.hz);              perfDash.end();
    perfSerial.begin(); serial_hal_update();                      perfSerial.end();
    perf_report();
}
