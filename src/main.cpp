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
#include "esp_log.h"

static const char* TAG = "perf";
static HzTracker   loopHz;

static volatile bool rpmPulse = false;
static void IRAM_ATTR onRpmPulse() { rpmPulse = true; }

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

    pinMode(PIN_RPM, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RPM), onRpmPulse, FALLING);
    ESP_LOGI(TAG, "RPM sensor ready on pin %d", PIN_RPM);

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

    perfImu.begin();  imu_update();                           perfImu.end();
    perfUwb.begin();  uwb_update();                           perfUwb.end();
    const UWBReading&    uwb  = uwb_get();
    const ImuData&       imu  = imu_get(); 
    perfNav.begin();  nav_update(uwb, imu);                        perfNav.end();
    const NavData&       nav  = nav_get();
    
    perfCtrl.begin(); control_update(nav, imu);                        perfCtrl.end();
    const ControlOutput& ctrl = control_get();
    perfOled.begin(); oled_update(loopHz.hz, nav, ctrl, imu); perfOled.end();
    perfWifi.begin(); wifi_update();                          perfWifi.end();

    if (rpmPulse) {
        rpmPulse = false;
        ESP_LOGI("rpm", "pulse detected");
    }

    perf_report(loopHz.hz);
}
