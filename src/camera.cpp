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

void camera_init() {
    Wire1.begin(PIN_CAMERA_SDA, PIN_CAMERA_SCL);
    Wire1.beginTransmission(CAMERA_I2C_ADDR);
    bool present = Wire1.endTransmission() == 0;
    if (present) ESP_LOGI(TAG, "✅ Camera ready at I2C 0x%02X", CAMERA_I2C_ADDR);
    else         ESP_LOGE(TAG, "❌ Camera not found at I2C 0x%02X", CAMERA_I2C_ADDR);
}

void camera_update() {
    float dt;
    if (!_gate.tick(dt)) return;

    const int PAYLOAD = 9;
    Wire1.requestFrom((uint8_t)CAMERA_I2C_ADDR, (uint8_t)PAYLOAD);
    if (Wire1.available() < PAYLOAD) {
        ESP_LOGW(TAG, "⚠️ short read (%d/9 bytes)", Wire1.available());
        while (Wire1.available()) Wire1.read();
        return;
    }

    uint8_t buf[PAYLOAD];
    for (int i = 0; i < PAYLOAD; i++) buf[i] = Wire1.read();

    bool found = buf[0] != 0;
    float posX, posY;
    memcpy(&posX, &buf[1], 4);
    memcpy(&posY, &buf[5], 4);

    _cameraData.found = found;
    if (found) {
        _cameraData.posX = posX;
        _cameraData.posY = posY;
    }
    _cameraData.timestamp = millis();
}

const CameraData& camera_get() { return _cameraData; }
