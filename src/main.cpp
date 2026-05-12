#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "oled.h"
#include "imu.h"

float lps = 0.0;
float seconds = 0.0;

// Function to track the speed of the main loop.
void updateLPS()
{
    static uint32_t count = 0;
    static uint32_t lastSecond = 0;

    count++;
    uint32_t now = millis();
    if (now - lastSecond >= 1000) {
        lps = count * 1000.0f / (now - lastSecond);
        count = 0;
        lastSecond = now;
    }
}

// Function to report the uptime and LPS to the serial monitor
void reportSerial()
{
    static uint32_t lastSerialReport = 0;
    if (millis() - lastSerialReport < SERIAL_REPORT_INTERVAL_MS)
        return;
    lastSerialReport = millis();

    Serial.print("Uptime: ");
    Serial.print(seconds, 2);
    Serial.print("s  LPS: ");
    Serial.print(lps / 1000.0, 2);
    Serial.print("K");
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

    Wire.begin(PIN_SDA, PIN_SCL);
    oled_init();
    imu_init();
    uwb_test();
    uwb_test();
    Serial.println("Starting Main Loop...");
}

void loop()
{
    seconds = millis() / 1000.0;
    updateLPS();
    imu_update();
    // reportSerial();
    oled_update(seconds, lps);
}
