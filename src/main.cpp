#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "utils.h"
#include "oled.h"
#include "imu.h"
#include "esp_log.h"

static const char *TAG = "main";

static HzTracker loopHz;

void reportSerial()
{
    static uint32_t lastSerialReport = 0;
    if (millis() - lastSerialReport < SERIAL_REPORT_INTERVAL_MS)
        return;
    lastSerialReport = millis();

    Serial.print("s  LPS: ");
    Serial.print(loopHz.hz);
    Serial.println("");

    const ImuData& imu = imu_get();
    Serial.print("  Y:");
    Serial.print(imu.yaw, 1);
    Serial.print(" P:");
    Serial.print(imu.pitch, 1);
    Serial.print(" R:");
    Serial.print(imu.roll, 1);
    Serial.print("  Cal:");
    Serial.print(imu.cal_rot);
    Serial.print(imu.cal_gyro);
    Serial.println(imu.cal_accel);
}

void uwb_test() {
    Serial.println("Sending AT to UWB...");
    Serial0.println("AT");
    delay(2000);
    if (Serial0.available()) {
        while (Serial0.available()) {
            Serial.print((char)Serial0.read());
        }
        Serial.println();
    } else {
        Serial.println("No response");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(3000);
    Serial.println("Setting Up...");

    Serial0.begin(9600, SERIAL_8N1, PIN_UWB_LEFT_RX, PIN_UWB_LEFT_TX);

    oled_init();
    imu_init();
    // uwb_test();

    Serial.println("Starting Main Loop...");
}

void loop()
{
    loopHz.update();
    imu_update();
    // reportSerial();
    oled_update(loopHz.hz);
}
