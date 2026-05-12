#include "oled.h"
#include "imu.h"
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
        ESP_LOGE(TAG, "OLED not found");
        while (true);
    }
    ESP_LOGI(TAG, "OLED ready");
    display.clearDisplay();
    display.display();
}

static void drawHeadingArrow(float yaw,boolean imuValid) {
    const int cx = 96, cy = 32, r = 28;
    const int bodyLen = 18, headLen = 6, headWidth = 4;

    display.drawCircle(cx, cy, r, SSD1306_WHITE);
    if (!imuValid) return;
    float angle = -(yaw - 90.0f) * M_PI / 180.0f;
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

void oled_update(float lps) {
    if (millis() - lastDisplayUpdate < OLED_UPDATE_INTERVAL_MS) return;
    lastDisplayUpdate = millis();
    oledHz.update();

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    const ImuData& imu = imu_get();

    display.setCursor(0, 0);
    display.print("LPS:");
    display.print(lps / 1000.0, 2);
    display.println("K");

    display.print("OLED:");
    display.println(oledHz.hz, 0);

    display.print("IMU:");
    display.println(imu.update_hz);

    display.print("LAT:");
    display.print(imu.latency_us / 1000.0f, 1);
    display.println("ms");

    drawHeadingArrow(imu.yaw,imu.update_hz);

    display.display();
}
