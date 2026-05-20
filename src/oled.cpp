// SSD1306 128x64 OLED display. Renders IMU stats, nav state, throttle/steering values, and a heading arrow.
#include "oled.h"
#include "nav.h"
#include "control.h"
#include "imu.h"
#include "config.h"
#include "utils.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "oled";

static Adafruit_SSD1306 _oledDisplay(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static unsigned long lastDisplayUpdate = 0;
static HzTracker _oledHz;

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

static void screen_1(float lps, const NavData& nav, const ControlOutput& output, const ImuData& imu) {
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
    // if (isnan(nav.relativeAngle)) {
    //     _oledDisplay.println("--");
    // } else {
    //     _oledDisplay.print(nav.relativeAngle, 0);
    //     _oledDisplay.println();
    // }

    _oledDisplay.print("NAV:");
    _oledDisplay.print(nav.updateHz, 2);
    _oledDisplay.println("hz");

    drawHeadingArrow(nav.relativeAngle, nav.distanceCm, nav.timestamp);

    if (!isnan(nav.distanceCm)) {
        char distBuf[8];
        snprintf(distBuf, sizeof(distBuf), "%dcm", (int)nav.distanceCm);
        _oledDisplay.setTextSize(2);
        int textW = strlen(distBuf) * 12;
        _oledDisplay.setCursor(96 - textW / 2, 40);
        _oledDisplay.print(distBuf);
        _oledDisplay.setTextSize(1);
    }

    // last line: nav state, throttle, steering
    _oledDisplay.setCursor(0, 56);
    _oledDisplay.print(nav.state == NavState::FOLLOW_ME ? "OK" : "XX");
    _oledDisplay.print(" T:");
    _oledDisplay.print(output.throttle*100, 0);
    _oledDisplay.print(" S:");
    _oledDisplay.print(output.steering*100, 0);
}

static void screen_2(float lps, const NavData& nav, const ControlOutput& output, const ImuData& imu) {
    if (nav.state == NavState::FOLLOW_ME)
        _oledDisplay.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SSD1306_WHITE);

    const int barW = 12, barMargin = 2, barSpacing = 4;

    // Throttle bar: chevrons (^) stacked from bottom, top = THROTTLE_SCALE
    const int chevW = barW, chevH = 3, chevStep = 5;
    int throttleH = (int)(constrain(output.throttle / THROTTLE_SCALE, 0.0f, 1.0f) * (OLED_HEIGHT - 2));
    for (int y = OLED_HEIGHT - 1 - chevH; y >= OLED_HEIGHT - 1 - throttleH; y -= chevStep) {
        _oledDisplay.drawLine(barMargin,               y + chevH, barMargin + chevW / 2, y,         SSD1306_WHITE);
        _oledDisplay.drawLine(barMargin + chevW / 2,   y,         barMargin + chevW,     y + chevH, SSD1306_WHITE);
    }

    // Heading age bar: fills from bottom, top = UWB_STALE_HEADING_MS
    const int ageBarX = barMargin + barW + barSpacing;
    unsigned long headingAge = nav.timestamp > 0 ? millis() - nav.timestamp : UWB_STALE_HEADING_MS;
    int ageBarH = constrain((int)((long)headingAge * (OLED_HEIGHT - 2) / UWB_STALE_HEADING_MS), 0, OLED_HEIGHT - 2);
    if (ageBarH > 0)
        _oledDisplay.fillRect(ageBarX, OLED_HEIGHT - 1 - ageBarH, barW, ageBarH, SSD1306_WHITE);

    // Compass calibration — centered in gap between bars and circle (x=30..68)
    _oledDisplay.setTextSize(1);
    _oledDisplay.setTextColor(SSD1306_WHITE);
    _oledDisplay.setCursor(40, 24);
    _oledDisplay.print("CAL");
    _oledDisplay.setCursor(40, 33);
    _oledDisplay.print(imu.cal_rot);
    _oledDisplay.print("/3");

    drawHeadingArrow(nav.relativeAngle, nav.distanceCm, nav.timestamp);

    _oledDisplay.setTextSize(2);
    _oledDisplay.setTextColor(SSD1306_WHITE);
    if (!isnan(nav.distanceCm) && nav.distanceCm >= 0) {
        char distBuf[8];
        snprintf(distBuf, sizeof(distBuf), "%dcm", (int)nav.distanceCm);
        int textW = strlen(distBuf) * 12;
        _oledDisplay.setCursor(96 - textW / 2, 38);
        _oledDisplay.print(distBuf);
    }
}

void oled_update(float lps) {
    const NavData&       nav    = nav_get();
    const ControlOutput& output = control_get();
    const ImuData&       imu    = imu_get();
    if (millis() - lastDisplayUpdate < OLED_UPDATE_INTERVAL_MS) return;
    lastDisplayUpdate = millis();
    _oledHz.update();

    _oledDisplay.clearDisplay();
    screen_2(lps, nav, output, imu);
    _oledDisplay.display();
}
