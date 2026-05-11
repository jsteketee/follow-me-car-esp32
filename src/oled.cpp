#include "oled.h"
#include "imu.h"
#include "config.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

void oled_update(float seconds, float lps) {
    if (millis() - lastDisplayUpdate < 100) return;
    lastDisplayUpdate = millis();

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.print("Uptime:  ");
    display.print(seconds, 1);
    display.print("s");

    display.setCursor(0, 11);
    display.print("LPS:     ");
    display.print(lps / 1000.0, 2);
    display.print("K");

    const ImuData& imu = imu_get();

    display.setCursor(0, 22);
    display.print("Y:");
    display.print(imu.yaw, 0);
    display.print(" P:");
    display.print(imu.pitch, 0);
    display.print(" R:");
    display.print(imu.roll, 0);

    display.setCursor(0, 33);
    display.print("Ax:");
    display.print(imu.ax, 1);
    display.print(" Ay:");
    display.print(imu.ay, 1);
    display.print(" Az:");
    display.print(imu.az, 1);

    display.setCursor(0, 44);
    display.print("Gx:");
    display.print(imu.gx, 1);
    display.print(" Gy:");
    display.print(imu.gy, 1);
    display.print(" Gz:");
    display.print(imu.gz, 1);

    display.setCursor(0, 55);
    display.print("Cal:");
    display.print(imu.cal_rot);
    display.print(imu.cal_gyro);
    display.print(imu.cal_accel);

    display.display();
}
