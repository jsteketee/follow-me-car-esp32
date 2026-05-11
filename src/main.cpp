#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "oled.h"
#include "imu.h"

uint32_t buckets[10] = {0};
int currentBucket = 0;
float lps = 0.0;
float seconds = 0.0;

// Function to track the speed of the main loop.
void updateLPS()
{
    int thisBucket = (millis() / 1000) % 10;
    if (thisBucket != currentBucket)
    {
        currentBucket = thisBucket;
        buckets[currentBucket] = 0;
    }
    buckets[currentBucket]++;

    uint32_t total = 0;
    for (int i = 0; i < 10; i++)
        total += buckets[i];
    lps = total / 10.0;
}

// Function to report the uptime and LPS to the serial monitor
void reportSerial()
{
    static uint32_t lastSerialReport = 0;
    if (millis() - lastSerialReport < 1000)
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

void setup()
{

    Serial.begin(115200);
    delay(3000);
    Serial.println("Setting Up...");
    Wire.begin(PIN_SDA, PIN_SCL);
    oled_init();
    imu_init();

    Serial.println("Starting Main Loop...");
}

void loop()
{
    seconds = millis() / 1000.0;
    updateLPS();
    imu_update();
    reportSerial();
    oled_update(seconds, lps);
}
