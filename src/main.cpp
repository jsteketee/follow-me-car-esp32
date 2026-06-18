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

static PerfTracker perfImu, perfUwb, perfNav, perfCtrl, perfOled, perfWifi;
static uint32_t    lastReport = 0;

static void perf_report(float lps) {
    if (millis() - lastReport < 1000) return;
    ESP_LOGI(TAG, "lps=%.0f  imu=%u/%u  uwb=%u/%u  nav=%u/%u  ctrl=%u/%u  oled=%u/%u  wifi=%u/%u  (avg/max us)",
        lps,
        perfImu.avg(),  perfImu.maxUs,
        perfUwb.avg(),  perfUwb.maxUs,
        perfNav.avg(),  perfNav.maxUs,
        perfCtrl.avg(), perfCtrl.maxUs,
        perfOled.avg(), perfOled.maxUs,
        perfWifi.avg(), perfWifi.maxUs);
    dashboard_set_perf({ perfImu.avg(),  perfImu.maxUs,
                         perfUwb.avg(),  perfUwb.maxUs,
                         perfNav.avg(),  perfNav.maxUs,
                         perfCtrl.avg(), perfCtrl.maxUs,
                         perfOled.avg(), perfOled.maxUs,
                         perfWifi.avg(), perfWifi.maxUs });
    perfImu.reset(); perfUwb.reset(); perfNav.reset();
    perfCtrl.reset(); perfOled.reset(); perfWifi.reset();
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
    // camera_init();
    fusion_init();

    nav_set_mode(DEFAULT_NAV_MODE);
    Serial.println("⭐⭐⭐⭐⭐ Setup Complete ⭐⭐⭐⭐⭐");
    if (UWB_DIAGNOSTICS_ON_STARTUP){
        Serial.println("⭐⭐⭐⭐⭐ Running UWB Diagnostics ⭐⭐⭐⭐⭐");
        uwb_run_diagnostics();
        Serial.println("⭐⭐⭐⭐⭐ Diagnostics Complete ⭐⭐⭐⭐⭐");
    }
}

void loop()
{
    loopHz.update();

    perfImu.begin();  imu_update();               perfImu.end();
    perfUwb.begin();  uwb_update();               perfUwb.end();
    // camera_update();
    fusion_update();
    perfNav.begin();  nav_update();               perfNav.end();
    perfCtrl.begin(); control_update();           perfCtrl.end();
    perfOled.begin(); oled_update(loopHz.hz);     perfOled.end();
    perfWifi.begin(); wifi_update();              perfWifi.end();
    rpm_update();
    dashboard_update(loopHz.hz);
    perf_report(loopHz.hz);
}
