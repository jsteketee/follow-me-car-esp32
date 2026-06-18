// SSD1306 128x64 OLED display. Renders IMU stats, nav state, throttle/steering values, and a heading arrow.
#include "oled.h"
#include "nav.h"
#include "fusion.h"
#include "control.h"
#include "imu.h"
#include "config.h"
#include "runtime_config.h"
#include "utils.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "oled";

static Adafruit_SSD1306 _oledDisplay(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static RateGate  _gate{ OLED_UPDATE_INTERVAL_MS };
static HzTracker _oledHz;
static uint32_t  _frameCount = 0;

void oled_init() {
    Wire.setPins(PIN_SDA, PIN_SCL);
    if (!_oledDisplay.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        ESP_LOGE(TAG, "❌ OLED not found");
        while (true);
    }
    Wire.setClock(400000);
    ESP_LOGI(TAG, "✅ OLED ready");
    _oledDisplay.clearDisplay();
    _oledDisplay.display();
}

static void drawHeadingArrow(float tagHeading, float tagDistCm, unsigned long navTimestamp) {
    const int cx = 96, cy = 32, r = 28;
    const int bodyLen = 18, headLen = 6, headWidth = 4;

    // top half only: 0x1=upper-left, 0x2=upper-right
    _oledDisplay.drawCircleHelper(cx, cy, r, 0x1 | 0x2, SSD1306_WHITE);

    if (isnan(tagHeading)) return;
    // 0° = straight ahead (up on screen), positive = right
    float angle = (tagHeading - 90.0f) * M_PI / 180.0f;
    float dx = cosf(angle), dy = sinf(angle);
    float px = -dy,         py = dx;

    int sx = cx + (int)(dx * r / 2), sy = cy + (int)(dy * r / 2); // start at midpoint
    int bx = cx + (int)(dx * bodyLen), by = cy + (int)(dy * bodyLen);
    int tx = bx + (int)(dx * headLen), ty = by + (int)(dy * headLen);
    int lx = bx + (int)(px * headWidth), ly = by + (int)(py * headWidth);
    int rx = bx - (int)(px * headWidth), ry = by - (int)(py * headWidth);

    _oledDisplay.drawLine(sx, sy, bx, by, SSD1306_WHITE);
    _oledDisplay.fillTriangle(tx, ty, lx, ly, rx, ry, SSD1306_WHITE);
}

static void screen_1(float lps, const NavData& nav, const Pose& fused, const ControlOutput& output, const ImuData& imu) {
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

    // _oledDisplay.print("TAG:");
    // if (isnan(fused.angle)) {
    //     _oledDisplay.println("--");
    // } else {
    //     _oledDisplay.print(fused.angle, 0);
    //     _oledDisplay.println();
    // }

    _oledDisplay.print("NAV:");
    _oledDisplay.print(nav.updateHz, 2);
    _oledDisplay.println("hz");

    drawHeadingArrow(fused.angle, fused.distanceCm, fused.timestamp);

    if (fused.distanceCm >= 0) {
        char distBuf[8];
        snprintf(distBuf, sizeof(distBuf), "%dcm", (int)fused.distanceCm);
        _oledDisplay.setTextSize(2);
        int textW = strlen(distBuf) * 12;
        _oledDisplay.setCursor(96 - textW / 2, 40);
        _oledDisplay.print(distBuf);
        _oledDisplay.setTextSize(1);
    }

    // last line: nav state, throttle, steering
    _oledDisplay.setCursor(0, 56);
    _oledDisplay.print(nav.mode == NavMode::FOLLOW_ME ? "OK" : "XX");
    _oledDisplay.print(" T:");
    _oledDisplay.print(output.throttle*100, 0);
    _oledDisplay.print(" S:");
    _oledDisplay.print(output.steering*100, 0);
}

static void screen_2(float lps, const NavData& nav, const Pose& fused, const ControlOutput& output) {
    if (nav.mode == NavMode::FOLLOW_ME && nav.sensorsValid)
        _oledDisplay.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SSD1306_WHITE);

    const int barW = 12, barMargin = 2, barSpacing = 4;
    // barFloor: bottom pixel of bars — reserves 8px text row + 1px gap beneath
    const int barFloor = OLED_HEIGHT - 10;

    // Throttle bar: chevrons (^) stacked from bottom, top = THROTTLE_SCALE
    const int chevW = barW, chevH = 3, chevStep = 5;
    int throttleH = (int)(constrain(output.throttle / THROTTLE_SCALE, 0.0f, 1.0f) * (barFloor + 1));
    for (int y = barFloor - chevH; y >= barFloor - throttleH; y -= chevStep) {
        _oledDisplay.drawLine(barMargin,               y + chevH, barMargin + chevW / 2, y,         SSD1306_WHITE);
        _oledDisplay.drawLine(barMargin + chevW / 2,   y,         barMargin + chevW,     y + chevH, SSD1306_WHITE);
    }

    // Uncertainty bar: fills from bottom, full height = FUSION_STALE_UNCERTAINTY
    const int uncBarX = barMargin + barW + barSpacing;
    int uncBarH = constrain((int)(fused.uncertainty / rtConfig.fusionStaleUncertainty * (barFloor + 1)), 0, barFloor + 1);
    if (uncBarH > 0)
        _oledDisplay.fillRect(uncBarX, barFloor - uncBarH + 1, barW, uncBarH, SSD1306_WHITE);

    drawHeadingArrow(fused.angle, fused.distanceCm, fused.timestamp);

    _oledDisplay.setTextSize(2);
    _oledDisplay.setTextColor(SSD1306_WHITE);
    if (!isnan(fused.distanceCm) && fused.distanceCm >= 0) {
        char distBuf[8];
        snprintf(distBuf, sizeof(distBuf), "%dcm", (int)fused.distanceCm);
        int textW = strlen(distBuf) * 12;
        _oledDisplay.setCursor(96 - textW / 2, 38);
        _oledDisplay.print(distBuf);
    }

    _oledDisplay.setTextSize(1);
    _oledDisplay.setTextColor(SSD1306_WHITE);
    _oledDisplay.setCursor(2, barFloor + 2);
    const char* modeStr =
        nav.mode == NavMode::FOLLOW_ME ? "FOLLOW" :
        nav.mode == NavMode::TEST      ? "TEST"   :
                                         "STOP";
    _oledDisplay.print(modeStr);
}

void oled_update(float lps) {
    const NavData&       nav    = nav_get();
    const Pose&     fused  = fusion_get();
    const ControlOutput& output = control_get();
    float dt;
    if (!_gate.tick(dt)) return;
    _oledHz.update();

    _frameCount++;
    _oledDisplay.clearDisplay();
    screen_2(lps, nav, fused, output);
    _oledDisplay.display();
}
