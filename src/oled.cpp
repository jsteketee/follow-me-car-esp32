// SSD1306 128x64 OLED display. Renders IMU stats, nav state, throttle/steering values, and a heading arrow.
#include "oled.h"
#include "uwb.h"
#include "control.h"
#include "imu.h"
#include "rpm.h"
#include "wifi_config.h"
#include "config.h"
#include "runtime_config.h"
#include "utils.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "oled";

static Adafruit_SSD1306 _oledDisplay(OLED_WIDTH, OLED_HEIGHT, &Wire1, -1);
static RateGate  _gate{ OLED_UPDATE_INTERVAL_MS };
static HzTracker _oledHz;
static uint32_t  _frameCount = 0;

// Snapshot of everything a frame renders, copied on the loop task and consumed by the
// render task on core 0 — the task never calls the module _get() functions directly,
// so cross-core reads of live module state can't tear mid-write.
struct OledSnapshot {
    float         lps;
    ControlMode   mode;
    UWBReading    uwb;
    ControlOutput output;
    RPMData       rpm;
    WifiInfo      wifi;
    uint32_t      imuTsMs;  // millis of the last IMU rotation report — drives the GYRO stale flag
};
static OledSnapshot _oledSnap;
static portMUX_TYPE _oledSnapMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t _oledRenderTask = nullptr;

// Render+push time per frame on core 0. Updated by the render task, read/reset once
// per window by perf_report on the loop task (guarded by _oledSnapMux).
static PerfTracker _oledRenderPerf;

// Returns the window's per-frame render+push timing (avg/max µs) and resets it.
void oled_render_perf(uint32_t& avgUs, uint32_t& maxUs) {
    taskENTER_CRITICAL(&_oledSnapMux);
    avgUs = _oledRenderPerf.avg();
    maxUs = _oledRenderPerf.maxUs;
    _oledRenderPerf.reset();
    taskEXIT_CRITICAL(&_oledSnapMux);
}

static void oled_render_task(void*);

void oled_init() {
    // Pin Wire1 before _oledDisplay.begin() — its internal wire->begin() takes
    // default pins if the controller isn't already started.
    Wire1.begin(PIN_OLED_SDA, PIN_OLED_SCL);
    if (!_oledDisplay.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.printf("[%s] ❌ OLED not found\n", TAG);
        while (true);
    }
    Wire1.setClock(400000);
    // Overflowing text must clip at the screen edge, not wrap: GFX's default wrap
    // restarts at x=0 on the next row, dropping stray glyphs into other regions.
    _oledDisplay.setTextWrap(false);
    Serial.printf("[%s] ✅ OLED ready\n", TAG);
    _oledDisplay.clearDisplay();
    _oledDisplay.display();

    // Render on core 0 (loop runs on core 1) so the ~25ms blocking I2C framebuffer
    // push in display() never stalls the control loop.
    xTaskCreatePinnedToCore(oled_render_task, "oled_render", 4096, nullptr, 1, &_oledRenderTask, 0);
}

static void drawHeadingArrow(float tagHeading) {
    // Pivot centered in the top-right box (x 79..127, y 0..49): the arrow's
    // 24px reach fits the box in every direction.
    const int cx = 103, cy = 24, r = 28;
    const int bodyLen = 18, headLen = 6, headWidth = 4;

    if (isnan(tagHeading)) return;
    // 0° = straight ahead (up on screen), positive = right
    float angle = (tagHeading - 90.0f) * M_PI / 180.0f;
    float dx = cosf(angle), dy = sinf(angle);
    float px = -dy,         py = dx;

    int sx = cx + (int)(dx * r / 4), sy = cy + (int)(dy * r / 4); // longer tail: start at quarter radius
    int bx = cx + (int)(dx * bodyLen), by = cy + (int)(dy * bodyLen);
    int tx = bx + (int)(dx * headLen), ty = by + (int)(dy * headLen);
    int lx = bx + (int)(px * headWidth), ly = by + (int)(py * headWidth);
    int rx = bx - (int)(px * headWidth), ry = by - (int)(py * headWidth);

    _oledDisplay.drawLine(sx, sy, bx, by, SSD1306_WHITE);
    _oledDisplay.fillTriangle(tx, ty, lx, ly, rx, ry, SSD1306_WHITE);
}

static void screen_1(float lps, ControlMode mode, const UWBReading& uwb, const ControlOutput& output, const ImuData& imu) {
    _oledDisplay.setTextColor(SSD1306_WHITE);
    _oledDisplay.setTextSize(1);

    _oledDisplay.setCursor(0, 0);
    _oledDisplay.print("LPS:");
    _oledDisplay.print(lps / 1000.0, 2);
    _oledDisplay.println("K");

    _oledDisplay.print("IMU:");
    _oledDisplay.println(imu.update_hz);

    _oledDisplay.print("IMU LAT:");
    _oledDisplay.print(imu.latency_us / 1000.0f, 0);
    _oledDisplay.println();

    _oledDisplay.print("HDG:");
    _oledDisplay.print(imu.yaw, 0);
    _oledDisplay.println();

    _oledDisplay.print("CAL:");
    _oledDisplay.print(imu.cal_rot);
    _oledDisplay.println("/3");

    drawHeadingArrow(uwb.angleDeg);

    if (uwb.distCm >= 0) {
        char distBuf[8];
        snprintf(distBuf, sizeof(distBuf), "%dcm", (int)uwb.distCm);
        _oledDisplay.setTextSize(2);
        int textW = strlen(distBuf) * 12;
        _oledDisplay.setCursor(96 - textW / 2, 40);
        _oledDisplay.print(distBuf);
        _oledDisplay.setTextSize(1);
    }

    // last line: control state (armed vs stopped), throttle, steering
    _oledDisplay.setCursor(0, 56);
    _oledDisplay.print(mode != ControlMode::STOPPED ? "OK" : "XX");
    _oledDisplay.print(" T:");
    _oledDisplay.print(output.throttle*100, 0);
    _oledDisplay.print(" S:");
    _oledDisplay.print(output.steering*100, 0);
}

static void screen_2(float lps, ControlMode mode, const UWBReading& uwb, const ControlOutput& output, const RPMData& rpm,
                     const WifiInfo& wifi, uint32_t imuTsMs) {
    const int barW = 12, barMargin = 2, barSpacing = 4;
    // barFloor: bottom pixel of bars — reserves 8px text row + 1px gap beneath
    const int barFloor = OLED_HEIGHT - 10;

    // Throttle bar: chevrons (^) stacked from bottom, full height = full [0,1] command
    // (same fraction the dashboard and screen 1 show; actuators scale to PWM downstream)
    const int chevW = barW, chevH = 3, chevStep = 5;
    int throttleH = (int)(constrain(output.throttle, 0.0f, 1.0f) * (barFloor + 1));
    for (int y = barFloor - chevH; y >= barFloor - throttleH; y -= chevStep) {
        _oledDisplay.drawLine(barMargin,               y + chevH, barMargin + chevW / 2, y,         SSD1306_WHITE);
        _oledDisplay.drawLine(barMargin + chevW / 2,   y,         barMargin + chevW,     y + chevH, SSD1306_WHITE);
    }

    // Target speed bar: outline only, full height = maxSpeedMph
    const int tgtBarX = barMargin + barW + barSpacing;
    float tgtNorm = rtConfig.maxSpeedMph > 0.0f
        ? constrain(output.targetSpeedMph / rtConfig.maxSpeedMph, 0.0f, 1.0f)
        : 0.0f;
    int tgtBarH = (int)(tgtNorm * (barFloor + 1));
    if (tgtBarH > 0)
        _oledDisplay.drawRect(tgtBarX, barFloor - tgtBarH + 1, barW, tgtBarH, SSD1306_WHITE);

    // Full-height outlines for both bars, drawn within each bar's existing width.
    _oledDisplay.drawRect(barMargin, 0, barW, barFloor + 1, SSD1306_WHITE);
    _oledDisplay.drawRect(tgtBarX,   0, barW, barFloor + 1, SSD1306_WHITE);

    // Arrow shows the commanded steering in every mode (SETPOINT PID output or DIRECT
    // effort): 0 = centered/up, ±90° = full lock. Negated to match the physical
    // steering direction as mounted — the dashboard's steer arrow applies the same
    // sign flip. Tag position lives on the dashboard.
    drawHeadingArrow(-output.steering * 90.0f);

    _oledDisplay.setTextSize(1);
    _oledDisplay.setTextColor(SSD1306_WHITE);
    // Distance (meters, top) and odometry (cm, bottom): bare size-2 values,
    // centered horizontally in the box (x 79..127, center 103).
    const int boxX = 79;
    char valBuf[8];
    if (!isnan(uwb.distCm) && uwb.distCm >= 0)
        snprintf(valBuf, sizeof(valBuf), "%.1f", uwb.distCm / 100.0f);
    else
        snprintf(valBuf, sizeof(valBuf), "-");  // no distance fix yet
    _oledDisplay.setTextSize(2);
    // Visible width: 12px per char advance, minus the 2px trailing gap of the last char.
    int distW = (int)strlen(valBuf) * 12 - 2;
    int distTX = 103 - distW / 2;
    if (distTX < boxX) distTX = boxX;
    _oledDisplay.setCursor(distTX, 17);
    _oledDisplay.print(valBuf);

    // Separator between the distance and odometry lines — extends to 2px shy of the box's right border.
    _oledDisplay.drawFastHLine(boxX, 35, 126 - boxX, SSD1306_WHITE);

    // Odometry — centimeters (distance calibration wants cm resolution) until the
    // 4-char box fills at 9999, then meters. The meters form keeps one decimal by
    // rendering it at size 1 ("999" + ".9" = 48px of the 49px box); from 1000 m the
    // integer part alone fills the box and the decimal is dropped.
    char odoBuf[8], odoFrac[4] = "";
    if (rpm.odometryCm < 10000.0f) {
        snprintf(odoBuf, sizeof(odoBuf), "%.0f", rpm.odometryCm);
    } else {
        float odoM = rpm.odometryCm / 100.0f;
        snprintf(odoBuf, sizeof(odoBuf), "%d", (int)odoM);
        if (odoM < 1000.0f) snprintf(odoFrac, sizeof(odoFrac), ".%d", (int)(odoM * 10.0f) % 10);
    }
    int odoW = (int)strlen(odoBuf) * 12 - 2 + (int)strlen(odoFrac) * 6;
    int odoTX = 103 - odoW / 2;
    if (odoTX < boxX) odoTX = boxX;
    _oledDisplay.setCursor(odoTX, 38);
    _oledDisplay.print(odoBuf);
    if (odoFrac[0]) {
        // Decimal at size 1, bottom-aligned to the size-2 digits (glyph rows 46–53 vs 38–53).
        _oledDisplay.setTextSize(1);
        _oledDisplay.setCursor(_oledDisplay.getCursorX(), 46);
        _oledDisplay.print(odoFrac);
    }
    _oledDisplay.setTextSize(1);

    _oledDisplay.setTextSize(1);
    _oledDisplay.setTextColor(SSD1306_WHITE);

    // Bar labels, aligned under each bar: throttle, set speed
    _oledDisplay.setCursor(barMargin, barFloor + 2);
    _oledDisplay.print("Tr");
    _oledDisplay.setCursor(tgtBarX, barFloor + 2);
    _oledDisplay.print("Ss");

    // Vertical divider centered between the Flag column (ends x=74) and the distance text (starts ~x=83).
    _oledDisplay.drawFastVLine(78, 0, OLED_HEIGHT, SSD1306_WHITE);

    // Horizontal divider off the vertical bar, 1px of separation above the bottom-aligned mode text.
    _oledDisplay.drawFastHLine(78, 54, OLED_WIDTH - 78, SSD1306_WHITE);

    // Flag label on the bottom line with a bar above it; COG sits on top of that bar
    // when active. The bar spans the full flag column — bars' right edge to the
    // vertical divider — with a 4px margin each side (matching the divider-side
    // margin); "Flag" centers beneath it.
    const int flagL = tgtBarX + barW + 4;
    const int flagR = 74;  // 4px short of the divider at x=78
    const int flagW = flagR - flagL + 1;
    _oledDisplay.drawFastHLine(flagL, barFloor, flagW, SSD1306_WHITE);
    _oledDisplay.setCursor(flagL + (flagW - 23) / 2, barFloor + 2);  // "Flag" = 4 chars × 6px − 1px trailing gap
    _oledDisplay.print("Flag");
    // Flag indicators, left-aligned to the bar's edge: WiFi pinned at the column top,
    // fault flags (blinking " *", ~1 Hz) stacked upward from the bar.
    // WiFi flag: WIFI-J = joined a network (STA), WIFI-H = hosting the fallback AP;
    // absent while still trying to connect.
    if (wifi.online) {
        _oledDisplay.setCursor(flagL, 0);
        _oledDisplay.print(wifi.sta ? "WIFI-J" : "WIFI-H");
    }
    // GYRO flag: no IMU rotation report for >100ms (or none since boot) — heading is
    // stale and the SETPOINT steering PID is flying blind.
    if (imuTsMs == 0 || millis() - imuTsMs > 100) {
        _oledDisplay.setCursor(flagL, barFloor - 18);
        _oledDisplay.print("GYRO");
        if ((millis() / 500) % 2 == 0) _oledDisplay.print(" *");
    }
    if (rpm.cogging) {
        _oledDisplay.setCursor(flagL, barFloor - 9);
        _oledDisplay.print("COG");
        if ((millis() / 500) % 2 == 0) _oledDisplay.print(" *");
    }

    // Control mode — horizontally centered in the bottom-right box, bottom aligned to the screen edge.
    const char* modeStr =
        mode == ControlMode::SETPOINT ? "SETPOINT" :
        mode == ControlMode::DIRECT ? "DIRECT" :
                                      "STOP";
    _oledDisplay.setCursor(79 + (OLED_WIDTH - 79 - (int)strlen(modeStr) * 6) / 2, 56);
    _oledDisplay.print(modeStr);
}

// Boot splash: WiFi connection progress, then SSID + dashboard IP. Shown until
// OLED_WIFI_SPLASH_MS after the network comes up; returns false once expired.
static bool screen_wifi_splash(const WifiInfo& wifi) {
    if (wifi.online && millis() - wifi.onlineSinceMs >= OLED_WIFI_SPLASH_MS) return false;

    _oledDisplay.setTextSize(1);
    _oledDisplay.setTextColor(SSD1306_WHITE);
    _oledDisplay.setCursor(0, 0);
    _oledDisplay.print("Follow Me Car");
    _oledDisplay.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);

    _oledDisplay.setCursor(0, 18);
    if (!wifi.online) {
        _oledDisplay.print("WiFi: trying");
        // Animated trailing dots (~2 Hz) so a hung connection attempt is visibly alive.
        for (uint32_t i = 0; i <= (millis() / 500) % 3; i++) _oledDisplay.print(".");
        _oledDisplay.setCursor(0, 30);
        _oledDisplay.print(wifi.ssid);
    } else {
        _oledDisplay.print(wifi.sta ? "WiFi: " : "own AP: ");
        _oledDisplay.print(wifi.ssid);
        _oledDisplay.setCursor(0, 36);
        _oledDisplay.print("Dashboard:");
        _oledDisplay.setCursor(0, 48);
        _oledDisplay.print("http://");
        _oledDisplay.print(wifi.ip);
    }
    return true;
}

// Core-0 render task: waits for a snapshot notification, then draws and pushes the
// frame. The blocking display() I2C transfer happens here, off the loop task.
static void oled_render_task(void*) {
    OledSnapshot snap;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        taskENTER_CRITICAL(&_oledSnapMux);
        snap = _oledSnap;
        taskEXIT_CRITICAL(&_oledSnapMux);

        _oledRenderPerf.begin();
        _oledDisplay.clearDisplay();
        if (!screen_wifi_splash(snap.wifi)) {
            screen_2(snap.lps, snap.mode, snap.uwb, snap.output, snap.rpm, snap.wifi, snap.imuTsMs);
        }
        _oledDisplay.display();
        taskENTER_CRITICAL(&_oledSnapMux);
        _oledRenderPerf.end();
        taskEXIT_CRITICAL(&_oledSnapMux);
    }
}

// Loop-task side: on each 200ms gate tick, snapshot the current module state and
// wake the render task. Costs microseconds; all drawing/I2C happens on core 0.
void oled_update(float lps) {
    float dt;
    if (!_gate.tick(dt)) return;
    _oledHz.update();
    _frameCount++;

    taskENTER_CRITICAL(&_oledSnapMux);
    _oledSnap.lps    = lps;
    _oledSnap.mode   = control_mode();
    _oledSnap.uwb    = uwb_get();
    _oledSnap.output = control_get();
    _oledSnap.rpm    = rpm_get();
    _oledSnap.wifi   = wifi_get();
    _oledSnap.imuTsMs = imu_get().timestamp;
    taskEXIT_CRITICAL(&_oledSnapMux);

    if (_oledRenderTask) xTaskNotifyGive(_oledRenderTask);
}
