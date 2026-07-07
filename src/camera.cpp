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
static RateGate   _retryGate{ 2000 };  // probe interval while waiting for camera to boot
static bool       _ready = false;

// Probes the I2C bus and logs every responding address.
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
    // Wire already started by oled_init(); pull-ups provided by OLED/IMU breakouts.
    // Single probe — don't block; camera_update() retries every 2s if not found.
    i2c_scan();
    Wire.beginTransmission(CAMERA_I2C_ADDR);
    _ready = (Wire.endTransmission() == 0);
    if (_ready) {
        ESP_LOGI(TAG, "✅ Camera ready at I2C 0x%02X", CAMERA_I2C_ADDR);
    } else {
        ESP_LOGW(TAG, "Camera not found at 0x%02X — will retry every 2s", CAMERA_I2C_ADDR);
    }
    // Always return true so camera_update() runs and can pick up a late-booting camera.
    return true;
}

bool camera_update() {
    if (!_ready) {
        // XIAO takes ~8s to boot; keep probing until it appears.
        float dt;
        if (!_retryGate.tick(dt)) return false;
        Wire.beginTransmission(CAMERA_I2C_ADDR);
        if (Wire.endTransmission() == 0) {
            _ready = true;
            ESP_LOGI(TAG, "✅ Camera found at I2C 0x%02X", CAMERA_I2C_ADDR);
        } else {
            ESP_LOGD(TAG, "Camera not yet ready at 0x%02X, retrying...", CAMERA_I2C_ADDR);
        }
        return false;
    }

    float dt;
    if (!_gate.tick(dt)) return false;

    const int PAYLOAD = 9;
    Wire.requestFrom((uint8_t)CAMERA_I2C_ADDR, (uint8_t)PAYLOAD);
    if (Wire.available() < PAYLOAD) {
        ESP_LOGW(TAG, "⚠️ short read (%d/9 bytes)", Wire.available());
        while (Wire.available()) Wire.read();
        return false;
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
    return true;
}

const CameraData& camera_get() { return _cameraData; }
