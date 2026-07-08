// DW3000 AOA UWB driver.
// Reads binary "carfollow" frames from a Makerfabs DW3000 anchor over UART.
// Frame format: 0x2A | length | payload (sn, addr, angle int32 LE, distCm int32 LE, ...) | XOR checksum | 0x23
// No handshake needed — anchor is pre-configured to USER_CMD 1 (binary mode) and paired with the tag.

#include "uwb.h"
#include "config.h"
#include "runtime_config.h"
#include "utils.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "uwb";

static HardwareSerial _dwSerial(1);

static UWBReading _uwbData = { .angleDeg = 0.0f, .distCm = -1.0f, .timestamp = 0 };

// --- Outlier rejection ---

static float _prevDistCm   = -1.0f; // last accepted distance, -1 = no baseline
static int   _rejectStreak = 0;     // consecutive outlier rejections

// --- Binary frame state machine (0x2A ... 0x23) ---

static const uint8_t FRAME_HEADER = 0x2A;
static const uint8_t FRAME_FOOTER = 0x23;

enum class BinState { WAIT_LENGTH, READ_PAYLOAD, WAIT_CHECKSUM, WAIT_FOOTER };

static BinState _binState       = BinState::WAIT_LENGTH;
static uint8_t  _binPayload[64] = {};
static uint8_t  _binLength      = 0;
static uint8_t  _binPayloadIdx  = 0;
static uint8_t  _binChecksum    = 0;


static void on_frame() {
    uint8_t calc = 0;
    for (int i = 0; i < _binLength; i++) calc ^= _binPayload[i];
    if (calc != _binChecksum) {
        ESP_LOGW(TAG, "checksum mismatch — frame discarded (calc=0x%02X recv=0x%02X)", calc, _binChecksum);
        return;
    }
    if (_binLength < 11) {
        ESP_LOGW(TAG, "frame too short (%u bytes) — discarded", _binLength);
        return;
    }

    int32_t rawAngle;
    int32_t rawDistCm;
    memcpy(&rawAngle,  &_binPayload[3], 4);
    memcpy(&rawDistCm, &_binPayload[7], 4);

    float angleDeg = -(float)rawAngle * DW3000_ANGLE_SCALE;
    float distCm   = (float)rawDistCm;

    // Outlier rejection on distance.
    bool accept = true;
    if (_prevDistCm >= 0.0f) {
        float jump = fabsf(distCm - _prevDistCm);
        if (jump > rtConfig.uwbOutlierRejectCm && _rejectStreak < UWB_OUTLIER_MAX_STREAK) {
            ESP_LOGW(TAG, "outlier rejected: %.0fcm (prev=%.0f jump=%.0f streak=%d)",
                distCm, _prevDistCm, jump, _rejectStreak + 1);
            _rejectStreak++;
            accept = false;
        }
    }

    if (accept) {
        _prevDistCm        = distCm;
        _rejectStreak      = 0;
        _uwbData.angleDeg  = angleDeg;
        _uwbData.distCm    = distCm;
        _uwbData.timestamp = millis();
        ESP_LOGD(TAG, "angle=%.1f°  dist=%.0fcm", angleDeg, distCm);
    }
}

static void process_byte(uint8_t b) {
    static bool _inFrame = false;

    if (!_inFrame) {
        if (b == FRAME_HEADER) {
            _inFrame   = true;
            _binState  = BinState::WAIT_LENGTH;
        }
        return;
    }

    switch (_binState) {
        case BinState::WAIT_LENGTH:
            _binLength     = b;
            _binPayloadIdx = 0;
            if (_binLength > 0 && _binLength <= (uint8_t)sizeof(_binPayload)) {
                _binState = BinState::READ_PAYLOAD;
            } else {
                ESP_LOGW(TAG, "implausible frame length %u — resyncing", _binLength);
                _inFrame = false;
            }
            break;

        case BinState::READ_PAYLOAD:
            _binPayload[_binPayloadIdx++] = b;
            if (_binPayloadIdx >= _binLength) _binState = BinState::WAIT_CHECKSUM;
            break;

        case BinState::WAIT_CHECKSUM:
            _binChecksum = b;
            _binState    = BinState::WAIT_FOOTER;
            break;

        case BinState::WAIT_FOOTER:
            if (b == FRAME_FOOTER) {
                on_frame();
            } else {
                ESP_LOGW(TAG, "expected footer 0x23, got 0x%02X — frame discarded", b);
            }
            _inFrame = false;
            break;
    }
}

// --- Public API ---

void uwb_init() {
    _dwSerial.begin(DW3000_BAUD, SERIAL_8N1, PIN_DW3000_RX, PIN_DW3000_TX);

    // Wait up to 2s for the first ranging frame. The anchor streams immediately on power-up,
    // so a timeout here means a wiring or power problem.
    uint32_t deadline = millis() + 2000;
    while (millis() < deadline) {
        while (_dwSerial.available()) process_byte((uint8_t)_dwSerial.read());
        if (_uwbData.timestamp > 0) break;
        delay(10);
    }

    if (_uwbData.timestamp > 0) {
        Serial.printf("[%s] ✅ DW3000 ready  angle=%.1f°  dist=%.0fcm\n", TAG, _uwbData.angleDeg, _uwbData.distCm);
    } else {
        Serial.printf("[%s] ❌ DW3000 no frames in 2s — check wiring (RX=GPIO%d TX=GPIO%d)\n", TAG, PIN_DW3000_RX, PIN_DW3000_TX);
    }
}

bool uwb_update() {
    unsigned long prevTs = _uwbData.timestamp;
    while (_dwSerial.available()) {
        process_byte((uint8_t)_dwSerial.read());
    }
    return _uwbData.timestamp != prevTs;
}

const UWBReading& uwb_get() {
    return _uwbData;
}
