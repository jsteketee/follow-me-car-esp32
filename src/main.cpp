// Wiring layer. Calls _update() on each module, fetches their output via _get(), and routes data to consumers as parameters.
#include <Arduino.h>
#include "config.h"
#include "utils.h"
#include "oled.h"
#include "imu.h"
#include "uwb.h"
#include "nav.h"
#include "control.h"
#include "wifi_config.h"
#include "rpm.h"
#include "dashboard.h"
#include "camera.h"
#include "fusion.h"
#include "esp_log.h"

static const char* TAG = "perf";
static HzTracker   loopHz;

// Execution time per module (avg/max µs over the reporting window).
static PerfTracker perfImu, perfUwb, perfCam, perfFusion, perfNav,
                   perfCtrl, perfOled, perfWifi, perfRpm, perfDash;

// New-data poll rate for actual sensors only (not fusion, nav, control, etc.).
// IMU poll rate is exposed via imu_get().update_hz (tracked internally by imu_update).
static HzTracker hzUwb, hzCam, hzRpm;

static uint32_t lastReport = 0;

static void perf_report(float lps) {
    if (millis() - lastReport < 1000) return;
    const ImuData& imu = imu_get();

    Serial.printf(
        "[loop perf] lps=%.0f"
        "  imu=%u/%u"
        "  uwb=%u/%u"
        "  cam=%u/%u"
        "  fus=%u/%u"
        "  nav=%u/%u"
        "  ctrl=%u/%u"
        "  oled=%u/%u"
        "  wifi=%u/%u"
        "  rpm=%u/%u"
        "  dash=%u/%u"
        "  (avg/max us)\n",
        lps,
        perfImu.avg(),    perfImu.maxUs,
        perfUwb.avg(),    perfUwb.maxUs,
        perfCam.avg(),    perfCam.maxUs,
        perfFusion.avg(), perfFusion.maxUs,
        perfNav.avg(),    perfNav.maxUs,
        perfCtrl.avg(),   perfCtrl.maxUs,
        perfOled.avg(),   perfOled.maxUs,
        perfWifi.avg(),   perfWifi.maxUs,
        perfRpm.avg(),    perfRpm.maxUs,
        perfDash.avg(),   perfDash.maxUs);

    Serial.printf(
        "[sensor perf] imu=%.0fHz  uwb=%.0fHz  cam=%.0fHz  rpm=%.0fHz\n",
        (float)imu.update_hz, hzUwb.hz, hzCam.hz, hzRpm.hz);

    dashboard_set_perf({ perfImu.avg(),    perfImu.maxUs,
                         perfUwb.avg(),    perfUwb.maxUs,
                         perfNav.avg(),    perfNav.maxUs,
                         perfCtrl.avg(),   perfCtrl.maxUs,
                         perfOled.avg(),   perfOled.maxUs,
                         perfWifi.avg(),   perfWifi.maxUs });

    perfImu.reset();    perfUwb.reset();  perfCam.reset();    perfFusion.reset();
    perfNav.reset();    perfCtrl.reset(); perfOled.reset();   perfWifi.reset();
    perfRpm.reset();    perfDash.reset();
    lastReport = millis();
}

void setup()
{
    delay(3000);
    Serial.begin(115200);
    Serial.println("⭐⭐⭐⭐⭐ Setting Up ⭐⭐⭐⭐⭐");

    //Init each device
    wifi_init();
    oled_init();
    imu_init();
    uwb_init();
    control_init();
    rpm_init();
    dashboard_init();
#ifndef CAMERA_DISABLED
    camera_init();
#endif
    fusion_init();

    nav_set_mode(DEFAULT_NAV_MODE);
    Serial.println("⭐⭐⭐⭐⭐ Setup Complete ⭐⭐⭐⭐⭐");
}

void loop()
{
    loopHz.update();

    perfImu.begin();    imu_update();                              perfImu.end();
    perfUwb.begin();    hzUwb.update(uwb_update());               perfUwb.end();
#ifndef CAMERA_DISABLED
    perfCam.begin();    hzCam.update(camera_update());            perfCam.end();
#endif
    perfFusion.begin(); fusion_update();                          perfFusion.end();
    perfNav.begin();    nav_update();                             perfNav.end();
    perfCtrl.begin();   control_update();                         perfCtrl.end();
    perfOled.begin();   oled_update(loopHz.hz);                   perfOled.end();
    perfWifi.begin();   wifi_update();                            perfWifi.end();
    perfRpm.begin();    hzRpm.update(rpm_update());               perfRpm.end();
    perfDash.begin();   dashboard_update(loopHz.hz);              perfDash.end();
    perf_report(loopHz.hz);
}
