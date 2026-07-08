// SSD1306 128x64 OLED display. Renders IMU stats, nav state, throttle/steering values, and a heading arrow.
#include "oled.h"
#include "nav.h"
#include "fusion.h"
#include "control.h"
#include "imu.h"
#include "rpm.h"
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
        Serial.printf("[%s] ❌ OLED not found\n", TAG);
        while (true);
    }
    Wire.setClock(400000);
    Serial.printf("[%s] ✅ OLED ready\n", TAG);
    _oledDisplay.clearDisplay();
    _oledDisplay.display();
}

static void drawHeadingArrow(float tagHeading, float tagDistCm, unsigned long navTimestamp) {
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
    // if (isnan(fused.fusedAngle)) {
    //     _oledDisplay.println("--");
    // } else {
    //     _oledDisplay.print(fused.fusedAngle, 0);
    //     _oledDisplay.println();
    // }

    _oledDisplay.print("NAV:");
    _oledDisplay.print(nav.updateHz, 2);
    _oledDisplay.println("hz");

    drawHeadingArrow(fused.fusedAngle, fused.distanceCm, fused.timestamp);

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

static void screen_2(float lps, const NavData& nav, const Pose& fused, const ControlOutput& output, const RPMData& rpm) {
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

    // Target speed bar: outline only, full height = maxSpeedMph
    const int tgtBarX = uncBarX + barW + barSpacing;
    float tgtNorm = rtConfig.maxSpeedMph > 0.0f
        ? constrain(output.targetSpeedMph / rtConfig.maxSpeedMph, 0.0f, 1.0f)
        : 0.0f;
    int tgtBarH = (int)(tgtNorm * (barFloor + 1));
    if (tgtBarH > 0)
        _oledDisplay.drawRect(tgtBarX, barFloor - tgtBarH + 1, barW, tgtBarH, SSD1306_WHITE);

    // Full-height outlines for all three bars, drawn within each bar's existing width.
    _oledDisplay.drawRect(barMargin, 0, barW, barFloor + 1, SSD1306_WHITE);
    _oledDisplay.drawRect(uncBarX,   0, barW, barFloor + 1, SSD1306_WHITE);
    _oledDisplay.drawRect(tgtBarX,   0, barW, barFloor + 1, SSD1306_WHITE);

    drawHeadingArrow(fused.fusedAngle, fused.distanceCm, fused.timestamp);

    _oledDisplay.setTextSize(1);
    _oledDisplay.setTextColor(SSD1306_WHITE);
    // Distance (meters, top) and odometry (cm, bottom): bare size-2 values,
    // centered horizontally in the box (x 79..127, center 103).
    const int boxX = 79;
    char valBuf[8];
    if (!isnan(fused.distanceCm) && fused.distanceCm >= 0)
        snprintf(valBuf, sizeof(valBuf), "%.1f", fused.distanceCm / 100.0f);
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

    // Odometry in centimeters — displayed for distance calibration
    char odoBuf[8];
    snprintf(odoBuf, sizeof(odoBuf), "%.0f", rpm.odometryCm);
    int odoW = (int)strlen(odoBuf) * 12 - 2;
    int odoTX = 103 - odoW / 2;
    if (odoTX < boxX) odoTX = boxX;
    _oledDisplay.setCursor(odoTX, 38);
    _oledDisplay.print(odoBuf);
    _oledDisplay.setTextSize(1);

    _oledDisplay.setTextSize(1);
    _oledDisplay.setTextColor(SSD1306_WHITE);

    // Bar labels, aligned under each bar: throttle, uncertainty, set speed
    _oledDisplay.setCursor(barMargin, barFloor + 2);
    _oledDisplay.print("Tr");
    _oledDisplay.setCursor(uncBarX, barFloor + 2);
    _oledDisplay.print("Uc");
    _oledDisplay.setCursor(tgtBarX, barFloor + 2);
    _oledDisplay.print("Ss");

    // Vertical divider centered between the Flag column (ends x=74) and the distance text (starts ~x=83).
    _oledDisplay.drawFastVLine(78, 0, OLED_HEIGHT, SSD1306_WHITE);

    // Horizontal divider off the vertical bar, 1px of separation above the bottom-aligned mode text.
    _oledDisplay.drawFastHLine(78, 54, OLED_WIDTH - 78, SSD1306_WHITE);

    // Flag label on the bottom line with a bar above it; COG sits on top of that bar when active.
    _oledDisplay.setCursor(50, barFloor + 2);
    _oledDisplay.print("Flag");
    _oledDisplay.drawFastHLine(50, barFloor, 24, SSD1306_WHITE);
    if (rpm.cogging) {
        _oledDisplay.setCursor(50, barFloor - 9);
        _oledDisplay.print("COG");
        // Slow-blinking asterisk after COG (~1 Hz, 500ms on/off).
        if ((millis() / 500) % 2 == 0) {
            _oledDisplay.print("*");
        }
    }

    // Nav mode — horizontally centered in the bottom-right box, bottom aligned to the screen edge.
    const char* modeStr =
        nav.mode == NavMode::FOLLOW_ME ? "FOLLOW" :
        nav.mode == NavMode::TEST      ? "TEST"   :
                                         "STOP";
    _oledDisplay.setCursor(79 + (OLED_WIDTH - 79 - (int)strlen(modeStr) * 6) / 2, 56);
    _oledDisplay.print(modeStr);
}

void oled_update(float lps) {
    const NavData&       nav    = nav_get();
    const Pose&          fused  = fusion_get();
    const ControlOutput& output = control_get();
    const RPMData&       rpm    = rpm_get();
    float dt;
    if (!_gate.tick(dt)) return;
    _oledHz.update();

    _frameCount++;
    _oledDisplay.clearDisplay();
    screen_2(lps, nav, fused, output, rpm);
    _oledDisplay.display();
}
