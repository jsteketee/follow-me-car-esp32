#include "oled.h"
#include "imu.h"
#include "config.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static unsigned long lastDisplayUpdate = 0;

void oled_init() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OLED not found");
        while (true);
    }
    display.clearDisplay();
    display.display();
}

static void drawHeadingArrow(float yaw) {
    const int cx = 96, cy = 32, r = 28;
    const int bodyLen = 18, headLen = 6, headWidth = 4;

    display.drawCircle(cx, cy, r, SSD1306_WHITE);

    float angle = (yaw - 90.0f) * M_PI / 180.0f;
    float dx = cosf(angle), dy = sinf(angle);
    float px = -dy,         py = dx;

    int bx = cx + (int)(dx * bodyLen), by = cy + (int)(dy * bodyLen);
    int tx = bx + (int)(dx * headLen), ty = by + (int)(dy * headLen);
    int lx = bx + (int)(px * headWidth), ly = by + (int)(py * headWidth);
    int rx = bx - (int)(px * headWidth), ry = by - (int)(py * headWidth);

    display.drawLine(cx, cy, bx, by, SSD1306_WHITE);
    display.fillTriangle(tx, ty, lx, ly, rx, ry, SSD1306_WHITE);
    display.fillCircle(cx, cy, 2, SSD1306_WHITE);
}

void oled_update(float seconds, float lps) {
    if (millis() - lastDisplayUpdate < OLED_UPDATE_INTERVAL_MS) return;
    lastDisplayUpdate = millis();

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    const ImuData& imu = imu_get();

    display.setCursor(0, 0);
    display.print("LPS:");
    display.print(lps / 1000.0, 2);
    display.print("K");

    drawHeadingArrow(imu.yaw);

    display.display();
}
