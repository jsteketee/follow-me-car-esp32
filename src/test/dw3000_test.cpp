// Bench validation sketch for the Makerfabs DW3000 AOA anchor board.
// Standalone entry point (env:dw3000-test) — does not touch the car's main.cpp or existing UWB driver.
// Pairs with a discovered tag (addtag/save), switches the anchor into its binary "carfollow" report
// mode (USER_CMD 1), and logs parsed angle/distance frames.
//
// Two serial ports are in play: `Serial` is the ESP32-S3's USB-CDC link to the computer running
// `pio device monitor` — every log line below goes there. `dwSerial` is the UART wire to the anchor
// board (TXD1/RXD1 header pins) and carries the actual DW3000 protocol; it is never shown raw, only
// re-logged through the ESP_LOG* calls in this file.
#include <Arduino.h>
#include "esp_log.h"

static const char* TAG = "dw3000_test";

// Anchor UART wiring: TXD1 (anchor) -> RX_PIN, RXD1 (anchor) -> TX_PIN, shared GND. Anchor powered separately via its own USB-C.
static const int RX_PIN = 4;
static const int TX_PIN = 5;

static HardwareSerial dwSerial(1);

static bool _paired = false; // set once we've sent addtag+save for a discovered tag

// --- Link-level bookkeeping, purely for bench-test visibility -----------------------------------------

static uint32_t _totalBytes            = 0; // every byte seen from dwSerial, regardless of what it parses as
static uint32_t _binaryFrameOkCount    = 0;
static uint32_t _binaryFrameBadCount   = 0; // footer or checksum mismatch
static uint32_t _jsonFrameCount        = 0;
static bool     _firstByteSeen         = false;
static uint32_t _lastByteMs            = 0;
static uint32_t _lastHeartbeatMs       = 0;

static const uint32_t HEARTBEAT_INTERVAL_MS = 5000;
static const uint32_t SILENCE_WARNING_MS    = 5000; // how long with zero bytes ever before we start nagging

// Logs a one-line summary of link health — counts, pairing state, and time since the last byte seen.
static void log_heartbeat() {
    uint32_t now = millis();
    if (now - _lastHeartbeatMs < HEARTBEAT_INTERVAL_MS) return;
    _lastHeartbeatMs = now;

    if (!_firstByteSeen) {
        ESP_LOGW(TAG, "HEARTBEAT: no bytes received from anchor yet (%lums since boot) — check wiring/power on TXD1/RXD1/GND",
            (unsigned long)now);
        return;
    }

    ESP_LOGI(TAG, "HEARTBEAT: bytes=%lu  jsonFrames=%lu  binFrames(ok/bad)=%lu/%lu  paired=%s  idle=%lums",
        (unsigned long)_totalBytes, (unsigned long)_jsonFrameCount,
        (unsigned long)_binaryFrameOkCount, (unsigned long)_binaryFrameBadCount,
        _paired ? "yes" : "no", (unsigned long)(now - _lastByteMs));
}

// --- Binary "carfollow" frame parsing (0x2A ... 0x23), enabled via USER_CMD 1 -----------------------

static const uint8_t FRAME_HEADER = 0x2A;
static const uint8_t FRAME_FOOTER = 0x23;

enum class BinState { WAIT_LENGTH, READ_PAYLOAD, WAIT_CHECKSUM, WAIT_FOOTER };

static BinState _binState       = BinState::WAIT_LENGTH;
static uint8_t  _binPayload[64] = {};
static uint8_t  _binLength      = 0;
static uint8_t  _binPayloadIdx  = 0;
static uint8_t  _binChecksum    = 0;

// Sends a bare command line to the anchor (matches the module's "CMD value\r\n" text protocol).
static void send_command(const char* cmd) {
    dwSerial.print(cmd);
    dwSerial.print("\r\n");
    ESP_LOGI(TAG, "-> %s", cmd);
}

// Prints a byte that doesn't belong to any known frame — boot text, command acks, etc.
static void log_raw_byte(uint8_t b) {
    if (b >= 0x20 && b < 0x7F) ESP_LOGI(TAG, "raw: %c", (char)b);
    else                       ESP_LOGI(TAG, "raw: 0x%02X", b);
}

// Parses one completed 31-byte carfollow frame and logs the fields we care about, including an explicit checksum verdict.
static void log_binary_frame() {
    uint8_t calc = 0;
    for (int i = 0; i < _binLength; i++) calc ^= _binPayload[i];
    bool ok = (calc == _binChecksum);

    uint8_t  sn    = _binPayload[0];
    uint16_t addr  = _binPayload[1] | (_binPayload[2] << 8);
    int32_t  angle;  memcpy(&angle,  &_binPayload[3], 4);
    int32_t  distCm; memcpy(&distCm, &_binPayload[7], 4);

    if (ok) {
        _binaryFrameOkCount++;
        ESP_LOGI(TAG, "FRAME OK  len=%u  sn=%u  addr=0x%04X  angle=%d  dist=%dcm  checksum=0x%02X",
            _binLength, sn, addr, (int)angle, (int)distCm, calc);
    } else {
        _binaryFrameBadCount++;
        ESP_LOGW(TAG, "FRAME BAD CHECKSUM  len=%u  calculated=0x%02X  received=0x%02X  (frame discarded)",
            _binLength, calc, _binChecksum);
    }
}

// Feeds one byte through the binary frame state machine; logs a frame once a footer arrives.
static void process_binary_byte(uint8_t b) {
    switch (_binState) {
        case BinState::WAIT_LENGTH:
            _binLength     = b;
            _binPayloadIdx = 0;
            if (_binLength > 0 && _binLength <= sizeof(_binPayload)) {
                ESP_LOGD(TAG, "binary frame: header seen, length=%u", _binLength);
                _binState = BinState::READ_PAYLOAD;
            } else {
                ESP_LOGW(TAG, "binary frame: implausible length %u, resyncing", _binLength);
                _binState = BinState::WAIT_LENGTH;
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
                log_binary_frame();
            } else {
                _binaryFrameBadCount++;
                ESP_LOGW(TAG, "binary frame: expected footer 0x23, got 0x%02X (frame discarded)", b);
            }
            _binState = BinState::WAIT_LENGTH;
            break;
    }
}

// --- Length-prefixed JSON frame parsing ("JS" + 4 hex-digit length + payload) ------------------------
// Used for the anchor's unsolicited "NewTag" discovery announcement.

enum class JsonState { WAIT_S, READ_LEN, READ_PAYLOAD };

static JsonState _jsonState        = JsonState::WAIT_S;
static char      _jsonLenChars[5]  = {};
static int       _jsonLenIdx       = 0;
static int       _jsonLen          = 0;
static char      _jsonPayload[512] = {};
static int       _jsonPayloadIdx   = 0;

// Extracts the tag's 64-bit ID from a "NewTag" payload and sends addtag+save to bind it — matches
// the command the vendor's Windows GUI sends when the user checks a tag's "Joined" box.
static void handle_new_tag(const char* id64) {
    char addr16[5];
    memcpy(addr16, id64 + 12, 4); // 16-bit addr is the low 4 hex digits of the 64-bit ID
    addr16[4] = '\0';

    ESP_LOGI(TAG, "PAIRING: discovered tag id64=%s addr16=%s — sending addtag+save", id64, addr16);

    char cmd[64];
    snprintf(cmd, sizeof(cmd), "addtag %s %s 0001 64 00", id64, addr16); // fastrate=1 (10Hz), useIMU=0 — matches GUI defaults
    send_command(cmd);
    delay(200);
    send_command("save");
    delay(200);

    _paired = true;
    ESP_LOGI(TAG, "PAIRING: complete — expecting ranging frames for addr16=%s from now on", addr16);
}

// Logs a completed JSON payload and pairs with the tag if it's a "NewTag" discovery message.
static void handle_json_payload(const char* payload) {
    _jsonFrameCount++;
    ESP_LOGI(TAG, "JSON frame #%lu: %s", (unsigned long)_jsonFrameCount, payload);

    const char* p = strstr(payload, "\"NewTag\":\"");
    if (p) {
        if (_paired) {
            ESP_LOGI(TAG, "JSON: NewTag seen again, already paired — ignoring");
        } else {
            char id64[17];
            memcpy(id64, p + strlen("\"NewTag\":\""), 16);
            id64[16] = '\0';
            handle_new_tag(id64);
        }
    }
}

// Feeds one byte through the JSON frame state machine; hands off the payload once fully read.
static void process_json_byte(uint8_t b) {
    switch (_jsonState) {
        case JsonState::WAIT_S:
            _jsonState  = JsonState::READ_LEN; // this byte is the "S" of "JS", already confirmed by the caller
            _jsonLenIdx = 0;
            break;

        case JsonState::READ_LEN:
            _jsonLenChars[_jsonLenIdx++] = (char)b;
            if (_jsonLenIdx == 4) {
                _jsonLenChars[4] = '\0';
                _jsonLen        = (int)strtol(_jsonLenChars, nullptr, 16);
                _jsonPayloadIdx = 0;
                if (_jsonLen <= 0 || _jsonLen >= (int)sizeof(_jsonPayload)) {
                    ESP_LOGW(TAG, "JSON frame: implausible length 0x%s, resyncing", _jsonLenChars);
                    _jsonState = JsonState::WAIT_S;
                } else {
                    ESP_LOGD(TAG, "JSON frame: header seen, length=%d", _jsonLen);
                    _jsonState = JsonState::READ_PAYLOAD;
                }
            }
            break;

        case JsonState::READ_PAYLOAD:
            _jsonPayload[_jsonPayloadIdx++] = (char)b;
            if (_jsonPayloadIdx >= _jsonLen) {
                _jsonPayload[_jsonPayloadIdx] = '\0';
                handle_json_payload(_jsonPayload);
                _jsonState = JsonState::WAIT_S;
            }
            break;
    }
}

// --- Top-level dispatch: routes each incoming byte to the JSON or binary parser, or logs it raw ------

enum class FrameType { NONE, JSON, BINARY };
static FrameType _frameType = FrameType::NONE;
static bool      _sawJ      = false; // true after seeing 'J', waiting to confirm the "JS" prefix

// Routes one incoming byte: starts a JSON or binary frame, feeds an in-progress one, or logs it raw.
static void process_byte(uint8_t b) {
    _totalBytes++;
    if (!_firstByteSeen) {
        _firstByteSeen = true;
        ESP_LOGI(TAG, "LINK OK: first byte received from anchor (0x%02X) — UART wiring confirmed", b);
    }
    _lastByteMs = millis();

    if (_frameType == FrameType::JSON) {
        process_json_byte(b);
        if (_jsonState == JsonState::WAIT_S) _frameType = FrameType::NONE; // frame just completed or resynced
        return;
    }
    if (_frameType == FrameType::BINARY) {
        process_binary_byte(b);
        if (_binState == BinState::WAIT_LENGTH) _frameType = FrameType::NONE; // frame just completed or resynced
        return;
    }

    if (_sawJ) {
        _sawJ = false;
        if (b == 'S') {
            ESP_LOGD(TAG, "JSON frame: \"JS\" prefix detected");
            _frameType = FrameType::JSON;
            process_json_byte(b);
            return;
        }
        log_raw_byte('J');
        log_raw_byte(b);
        return;
    }

    if (b == 'J') { _sawJ = true; return; }
    if (b == FRAME_HEADER) {
        ESP_LOGD(TAG, "binary frame: 0x2A header detected");
        _frameType = FrameType::BINARY;
        _binState  = BinState::WAIT_LENGTH;
        return;
    }
    log_raw_byte(b);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10); // wait for USB CDC host to open the port before printing anything
    ESP_LOGI(TAG, "======================================================");
    ESP_LOGI(TAG, "DW3000 bench test starting");
    ESP_LOGI(TAG, "Step 1: opening dwSerial (UART1) RX=GPIO%d TX=GPIO%d @115200", RX_PIN, TX_PIN);
    dwSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(500);

    ESP_LOGI(TAG, "Step 2: sending USER_CMD 1 (switch anchor to binary carfollow report mode)");
    send_command("USER_CMD 1"); // 0=JSON, 1=binary carfollow — independent of pairing, safe to set anytime
    delay(200);

    ESP_LOGI(TAG, "Step 3: sending save (persist USER_CMD setting to flash)");
    send_command("save");
    delay(200);

    ESP_LOGI(TAG, "Step 4: entering main loop — watching for NewTag discovery and carfollow frames");
    ESP_LOGI(TAG, "======================================================");

    _lastByteMs      = millis();
    _lastHeartbeatMs = millis();
}

void loop() {
    while (dwSerial.available()) {
        process_byte((uint8_t)dwSerial.read());
    }
    log_heartbeat();
}
