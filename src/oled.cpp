// SSD1306 128x64 OLED display. Renders IMU stats, nav state, throttle/steering values, and a heading arrow.
#include "oled.h"
#include "config.h"
#include "utils.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "oled";

static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static unsigned long lastDisplayUpdate = 0;
static HzTracker oledHz;

void oled_init() {
    Wire.setPins(PIN_SDA, PIN_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        ESP_LOGE(TAG, "❌ OLED not found");
        while (true);
    }
    ESP_LOGI(TAG, "✅ OLED ready");
    display.clearDisplay();
    display.display();
}

static void drawHeadingArrow(float tagHeading, float tagDistCm, unsigned long navTimestamp) {
    const int cx = 96, cy = 32, r = 28;
    const int bodyLen = 18, headLen = 6, headWidth = 4;

    // top half only: 0x1=upper-left, 0x2=upper-right
    display.drawCircleHelper(cx, cy, r, 0x1 | 0x2, SSD1306_WHITE);

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

    display.drawLine(sx, sy, bx, by, SSD1306_WHITE);
    display.fillTriangle(tx, ty, lx, ly, rx, ry, SSD1306_WHITE);
}

static void screen_1(float lps, const NavData& nav, const ControlOutput& output, const ImuData& imu) {
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.print("LPS:");
    display.print(lps / 1000.0, 2);
    display.println("K");

    display.print("IMU:");
    display.println(imu.update_hz);

    display.print("IMU LAT:");
    display.print(imu.latency_us / 1000.0f, 0);
    display.println();

    display.print("HDG:");
    display.print(imu.yaw, 0);
    display.println();

    display.print("CAL:");
    display.print(imu.cal_rot);
    display.println("/3");

    // display.print("TAG:");
    // if (isnan(nav.relativeAngle)) {
    //     display.println("--");
    // } else {
    //     display.print(nav.relativeAngle, 0);
    //     display.println();
    // }

    display.print("NAV:");
    display.print(nav.updateHz, 2);
    display.println("hz");

    drawHeadingArrow(nav.relativeAngle, nav.distanceCm, nav.timestamp);

    if (!isnan(nav.distanceCm)) {
        char distBuf[8];
        snprintf(distBuf, sizeof(distBuf), "%dcm", (int)nav.distanceCm);
        display.setTextSize(2);
        int textW = strlen(distBuf) * 12;
        display.setCursor(96 - textW / 2, 40);
        display.print(distBuf);
        display.setTextSize(1);
    }

    // last line: nav state, throttle, steering
    display.setCursor(0, 56);
    display.print(nav.state == NavState::VALID ? "OK" : "XX");
    display.print(" T:");
    display.print(output.throttle*100, 0);
    display.print(" S:");
    display.print(output.steering*100, 0);
}

static void screen_2(float lps, const NavData& nav, const ControlOutput& output, const ImuData& imu) {
    if (nav.state == NavState::VALID)
        display.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SSD1306_WHITE);

    const int barW = 12, barMargin = 2, barSpacing = 4;

    // Throttle bar: chevrons (^) stacked from bottom, top = THROTTLE_SCALE
    const int chevW = barW, chevH = 3, chevStep = 5;
    int throttleH = (int)(constrain(output.throttle / THROTTLE_SCALE, 0.0f, 1.0f) * (OLED_HEIGHT - 2));
    for (int y = OLED_HEIGHT - 1 - chevH; y >= OLED_HEIGHT - 1 - throttleH; y -= chevStep) {
        display.drawLine(barMargin,               y + chevH, barMargin + chevW / 2, y,         SSD1306_WHITE);
        display.drawLine(barMargin + chevW / 2,   y,         barMargin + chevW,     y + chevH, SSD1306_WHITE);
    }

    // Heading age bar: fills from bottom, top = UWB_STALE_HEADING_MS
    const int ageBarX = barMargin + barW + barSpacing;
    unsigned long headingAge = nav.timestamp > 0 ? millis() - nav.timestamp : UWB_STALE_HEADING_MS;
    int ageBarH = constrain((int)((long)headingAge * (OLED_HEIGHT - 2) / UWB_STALE_HEADING_MS), 0, OLED_HEIGHT - 2);
    if (ageBarH > 0)
        display.fillRect(ageBarX, OLED_HEIGHT - 1 - ageBarH, barW, ageBarH, SSD1306_WHITE);

    // Compass calibration — centered in gap between bars and circle (x=30..68)
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(40, 24);
    display.print("CAL");
    display.setCursor(40, 33);
    display.print(imu.cal_rot);
    display.print("/3");

    drawHeadingArrow(nav.relativeAngle, nav.distanceCm, nav.timestamp);

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    if (!isnan(nav.distanceCm) && nav.distanceCm >= 0) {
        char distBuf[8];
        snprintf(distBuf, sizeof(distBuf), "%dcm", (int)nav.distanceCm);
        int textW = strlen(distBuf) * 12;
        display.setCursor(96 - textW / 2, 38);
        display.print(distBuf);
    }
}

void oled_update(float lps, const NavData& nav, const ControlOutput& output, const ImuData& imu) {
    if (millis() - lastDisplayUpdate < OLED_UPDATE_INTERVAL_MS) return;
    lastDisplayUpdate = millis();
    oledHz.update();

    display.clearDisplay();
    screen_2(lps, nav, output, imu);
    display.display();
}
