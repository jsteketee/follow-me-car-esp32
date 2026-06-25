// Blob detection camera — I2C slave at CAMERA_I2C_ADDR.
// Polls at CAMERA_UPDATE_INTERVAL_MS; returns found flag and normalised blob position.
// Uses Wire1 (GPIO4/5) — separate bus from OLED/IMU on Wire (GPIO8/9).
#include "camera.h"
#include "config.h"
#include "utils.h"
#include "esp_log.h"
#include <Wire.h>

static const char* TAG = "camera";
static CameraData _cameraData = {};
static RateGate   _gate{ CAMERA_UPDATE_INTERVAL_MS };

static void i2c_scan() {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            found = true;
        }
    }
    if (!found) ESP_LOGW(TAG, "  No I2C devices found");
}

bool camera_init() {
    // Wire already started by oled_init(); pull-ups provided by OLED/IMU breakouts
    i2c_scan();

    // Retry for up to 15s — XIAO takes ~8s to boot and run esp_camera_init
    const int RETRIES = 30;
    for (int i = 0; i < RETRIES; i++) {
        Wire.beginTransmission(CAMERA_I2C_ADDR);
        if (Wire.endTransmission() == 0) {
            ESP_LOGI(TAG, "✅ Camera ready at I2C 0x%02X (attempt %d)", CAMERA_I2C_ADDR, i + 1);
            return true;
        }
        ESP_LOGD(TAG, "Camera not ready, retrying (%d/%d)...", i + 1, RETRIES);
        delay(500);
    }
    ESP_LOGE(TAG, "❌ Camera not found at I2C 0x%02X after %d attempts — disabled", CAMERA_I2C_ADDR, RETRIES);
    return false;
}

void camera_update() {
    float dt;
    if (!_gate.tick(dt)) return;

    const int PAYLOAD = 9;
    Wire.requestFrom((uint8_t)CAMERA_I2C_ADDR, (uint8_t)PAYLOAD);
    if (Wire.available() < PAYLOAD) {
        ESP_LOGW(TAG, "⚠️ short read (%d/9 bytes)", Wire.available());
        while (Wire.available()) Wire.read();
        return;
    }

    uint8_t buf[PAYLOAD];
    for (int i = 0; i < PAYLOAD; i++) buf[i] = Wire.read();

    bool found = buf[0] != 0;
    float posX, posY;
    memcpy(&posX, &buf[1], 4);
    memcpy(&posY, &buf[5], 4);
    // ESP_LOGI(TAG, "raw: found=%d  posX=%.3f  posY=%.3f  bytes=[%02X %02X%02X%02X%02X %02X%02X%02X%02X]",
    //     found, posX, posY,
    //     buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8]);

    _cameraData.found = found;
    if (found) {
        _cameraData.posX = posX;
        _cameraData.posY = posY;
    }
    _cameraData.timestamp = millis();
}

const CameraData& camera_get() { return _cameraData; }
