// UWB ranging driver for two RYUW122 anchors. Polls via raw AT commands (library ranging is broken),
// applies per-anchor Kalman filtering, and exposes distLeft/distRight (filtered) and distFast (raw avg).
//TODO: Rerwite to get rid of library calls.

#include "uwb.h"
#include "config.h"
#include "utils.h"
#include "wifi_config.h"
#include "esp_log.h"
#include <RYUW122.h>
#include <math.h>

static const char *TAG = "uwb";

static HardwareSerial uartFront(0);

enum class PollState  { IDLE, AWAITING };
enum class CyclePhase { WAIT, LEFT_PENDING, RIGHT_PENDING };

struct UWBAnchor {
    const char*     position;
    char            moduleName[16] = {};
    HardwareSerial& serial;
    PollState    pollState = PollState::IDLE;
    uint32_t     sentAt    = 0;
    char         buf[128]  = {};
    int          bufPos    = 0;
    KalmanFilter kalman    = {};
    float        rawDist   = -1.0f;
    float        prevDist     = -1.0f; // last accepted distance, -1 = no baseline yet
    int          rejectStreak = 0;     // consecutive outlier rejections
};

static UWBReading _uwbData = {};

// These are the anchors we will actually use for ranging calls. 
static UWBAnchor anchorLeft  { "left",  {}, Serial1  };
static UWBAnchor anchorRight { "right", {}, Serial2  };
static UWBAnchor anchorFront { "front", {}, uartFront };

//These are objects from the RYUW122 library, however, library funct. for ranging is broken. 
//Used to initialize and test the anchors, but not for actual ranging.
static RYUW122 uwb_left (PIN_UWB_LEFT_TX,  PIN_UWB_LEFT_RX,  &Serial1,  PIN_UWB_NRST);
static RYUW122 uwb_right(PIN_UWB_RIGHT_TX, PIN_UWB_RIGHT_RX, &Serial2,  PIN_UWB_NRST);
static RYUW122 uwb_front(PIN_UWB_FRONT_TX, PIN_UWB_FRONT_RX, &uartFront, PIN_UWB_NRST);


static void uwb_check_config(RYUW122& uwb) {
    char addr[9] = {0};
    uwb.getAddress(addr);
    ESP_LOGI(TAG, "Address:    %s", addr);

    char netId[9] = {0};
    uwb.getNetworkId(netId);
    ESP_LOGI(TAG, "Network ID: %s", netId);

    char uid[32] = {0};
    uwb.getUid(uid);
    ESP_LOGI(TAG, "UID:        %s", uid);

    RYUW122Mode mode = uwb.getMode();
    ESP_LOGI(TAG, "Mode:       %s", RYUW122Mode_description(mode).c_str());

    RYUW122BaudRate baud = uwb.getBaudRate();
    ESP_LOGI(TAG, "Baud:       %s", RYUW122BaudRate_description(baud).c_str());

    RYUW122RFChannel ch = uwb.getRfChannel();
    ESP_LOGI(TAG, "RF Channel: %s", RYUW122RFChannel_description(ch).c_str());

    RYUW122Bandwidth bw = uwb.getBandwidth();
    ESP_LOGI(TAG, "Bandwidth:  %s", RYUW122Bandwidth_description(bw).c_str());

    RYUW122RFPower pwr = uwb.getRfPower();
    ESP_LOGI(TAG, "RF Power:   %s", RYUW122RFPower_description(pwr).c_str());

    char fw[32] = {0};
    uwb.getFirmwareVersion(fw);
    ESP_LOGI(TAG, "Firmware:   %s", fw);

    int cal = uwb.getDistanceCalibration();
    ESP_LOGI(TAG, "Cal offset: %d cm", cal);
}
static bool uwb_read_address(UWBAnchor& anchor, char* out, size_t outLen);

static bool uwb_init_anchor(RYUW122& uwb, UWBAnchor& anchor) {
    uwb.begin();
    bool ok = uwb.test();
    uwb_read_address(anchor, anchor.moduleName, sizeof(anchor.moduleName));
    if (ok) {
        ESP_LOGI(TAG, "✅ [%s/%s] ready", anchor.position, anchor.moduleName);
        // uwb_check_config(uwb);
    } else {
        ESP_LOGE(TAG, "❌ [%s/%s] not responding", anchor.position, anchor.moduleName);
    }
    return ok;
}

// Parses a +ANCHOR_RCV= response line and returns the distance field in cm, or -1.
static int parse_anchor_rcv(const char* buf) {
    const char* p = strstr(buf, "+ANCHOR_RCV=");
    if (!p) return -1;
    p += strlen("+ANCHOR_RCV=");
    for (int i = 0; i < 3 && p; i++) p = strchr(p, ',') + 1;
    return p ? atoi(p) : -1;
}

// Blocking single poll — used only at init time for calibration.
// Returns distance in cm, or -1 on timeout.
static int uwb_poll_blocking(UWBAnchor& anchor) {
    while (anchor.serial.available()) anchor.serial.read();
    anchor.serial.print("AT+ANCHOR_SEND=TAG,4,TEST\r\n");

    uint32_t sentAt = millis();
    char buf[128]; int pos = 0; buf[0] = '\0';

    while (millis() - sentAt < UWB_RESPONSE_TIMEOUT_MS) {
        while (anchor.serial.available() && pos < 127) {
            char c = anchor.serial.read();
            if (c == '\n') {
                buf[pos] = '\0';
                if (strstr(buf, "+ANCHOR_RCV=")) {
                    return parse_anchor_rcv(buf);
                }
                pos = 0;
            } else if (c != '\r') {
                buf[pos++] = c;
            }
        }
    }
    return -1;
}

// Queries the module address via AT+ADDRESS? Returns true if successful.
static bool uwb_read_address(UWBAnchor& anchor, char* out, size_t outLen) {
    while (anchor.serial.available()) anchor.serial.read();
    anchor.serial.print("AT+ADDRESS?\r\n");

    uint32_t sentAt = millis();
    char buf[64]; int pos = 0; buf[0] = '\0';

    while (millis() - sentAt < UWB_RESPONSE_TIMEOUT_MS) {
        while (anchor.serial.available() && pos < 63) {
            char c = anchor.serial.read();
            if (c == '\n') {
                buf[pos] = '\0';
                const char* prefix = "+ADDRESS=";
                char* p = strstr(buf, prefix);
                if (p) {
                    strncpy(out, p + strlen(prefix), outLen - 1);
                    out[outLen - 1] = '\0';
                    return true;
                }
                pos = 0;
            } else if (c != '\r') {
                buf[pos++] = c;
            }
        }
    }
    return false;
}

static void uwb_test_anchor(UWBAnchor& anchor, uint32_t durationMs) {
    int polls = 0, successes = 0;
    int minDist = INT_MAX, maxDist = 0;
    float mean = 0, M2 = 0; // Welford's online variance
    uint32_t start = millis();

    ESP_LOGI(TAG, "🔍 [%s/%s] running %us test...", anchor.position, anchor.moduleName, durationMs / 1000);

    while (millis() - start < durationMs) {
        int dist = uwb_poll_blocking(anchor);
        polls++;
        if (dist >= 0) {
            successes++;
            float delta = dist - mean;
            mean += delta / successes;
            M2  += delta * (dist - mean);
            if (dist < minDist) minDist = dist;
            if (dist > maxDist) maxDist = dist;
        }
        delay(UWB_CAL_POLL_INTERVAL_MS);
    }

    float stddev = successes > 1 ? sqrtf(M2 / (successes - 1)) : 0;
    ESP_LOGI(TAG, "  [%s/%s] %d/%d ok (%.0f%%)  avg=%.0fcm  stddev=%.1fcm  min=%d  max=%d",
        anchor.position, anchor.moduleName, successes, polls, 100.0f * successes / polls,
        mean, stddev, successes > 0 ? minDist : -1, maxDist);
}

void uwb_run_diagnostics() {
    ESP_LOGI(TAG, "=== UWB Diagnostics Start ===");
    uwb_test_anchor(anchorLeft,  30000);
    uwb_test_anchor(anchorRight, 30000);
    ESP_LOGI(TAG, "=== UWB Diagnostics Complete ===");
}

static void uwb_calibrate_anchor(RYUW122& uwb, UWBAnchor& anchor, float knownDistCm, int numSamples) {
    if (uwb_poll_blocking(anchor) < 0) {
        ESP_LOGW(TAG, "⚠️ [%s/%s] calibration skipped — tag not found", anchor.position, anchor.moduleName);
        return;
    }

    int existingCal = uwb.getDistanceCalibration();
    int samples[numSamples];
    int count = 0;

    for (int i = 0; i < numSamples; i++) {
        int d = uwb_poll_blocking(anchor);
        if (d >= 0) samples[count++] = d;
        // Match normal poll interval so TWR conditions match steady-state operation.
        delay(UWB_CAL_POLL_INTERVAL_MS);
    }

    // Mean of all samples
    float sum = 0;
    for (int i = 0; i < count; i++) sum += samples[i];
    float mean = sum / count;

    // Standard deviation
    float variance = 0;
    for (int i = 0; i < count; i++) variance += (samples[i] - mean) * (samples[i] - mean);
    float sigma = sqrtf(variance / count);

    // Reject outliers (> 2σ from mean), recompute mean
    float filteredSum = 0; int filteredCount = 0;
    for (int i = 0; i < count; i++) {
        if (fabsf(samples[i] - mean) <= 2.0f * sigma) {
            filteredSum += samples[i];
            filteredCount++;
        }
    }

    if (filteredCount == 0) {
        ESP_LOGW(TAG, "⚠️ [%s/%s] calibration: all samples rejected as outliers, using raw mean", anchor.position, anchor.moduleName);
        filteredSum = sum; filteredCount = count;
    }

    float filteredMean = filteredSum / filteredCount;
    int error = (int)roundf(filteredMean - knownDistCm);

    int newCal = existingCal - error;
    ESP_LOGI(TAG, "✅ [%s/%s] calibration: %d/%d samples kept, mean=%.1f, known=%.1f, error=%d cm, cal %d→%d",
             anchor.position, anchor.moduleName, filteredCount, count, filteredMean, knownDistCm, error, existingCal, newCal);

    uwb.setDistanceCalibration(newCal);
}

void uwb_init() {
    uwb_init_anchor(uwb_left,  anchorLeft);
    uwb_init_anchor(uwb_right, anchorRight);
    uwb_init_anchor(uwb_front, anchorFront);
    if (UWB_CALIBRATE_ON_STARTUP) {
        uwb_calibrate_anchor(uwb_left,  anchorLeft,  UWB_CALIBRATION_DISTANCE_CM, UWB_CALIBRATION_SAMPLES);
        uwb_calibrate_anchor(uwb_right, anchorRight, UWB_CALIBRATION_DISTANCE_CM, UWB_CALIBRATION_SAMPLES);
    }
}

static void uwb_start_poll(UWBAnchor& anchor) {
    while (anchor.serial.available()) anchor.serial.read();
    anchor.serial.print("AT+ANCHOR_SEND=TAG,4,TEST\r\n");
    anchor.sentAt    = millis();
    anchor.bufPos    = 0;
    anchor.buf[0]    = '\0';
    anchor.pollState = PollState::AWAITING;
}

// Drains UART for one anchor and updates _uwbData when a response or timeout arrives.
static void uwb_update_anchor(UWBAnchor& anchor) {
    while (anchor.serial.available() && anchor.bufPos < 127) {
        char c = anchor.serial.read();
        if (c == '\n') {
            anchor.buf[anchor.bufPos] = '\0';
            if (strstr(anchor.buf, "+ANCHOR_RCV=")) {
                int dist = parse_anchor_rcv(anchor.buf);
                uint32_t latency = millis() - anchor.sentAt;
                bool valid = dist >= 0;

                // Outlier rejection: discard single-poll jumps larger than threshold.
                // After UWB_OUTLIER_MAX_STREAK consecutive rejections, force-accept so a
                // genuine large movement doesn't permanently block updates.
                bool headingStale = millis() - _uwbData.timestamp > UWB_STALE_HEADING_MS;
                if (valid && anchor.prevDist >= 0.0f && !headingStale) {
                    float jump = fabsf((float)dist - anchor.prevDist);
                    if (jump > UWB_OUTLIER_REJECT_CM && anchor.rejectStreak < UWB_OUTLIER_MAX_STREAK) {
                        ESP_LOGW(TAG, "⚠️ [%s/%s] outlier rejected: %dcm (prev=%.0fcm jump=%.0fcm streak=%d)",
                                 anchor.position, anchor.moduleName, dist, anchor.prevDist, jump, anchor.rejectStreak + 1);
                        anchor.rejectStreak++;
                        valid = false;
                    }
                }
                if (valid) {
                    anchor.prevDist     = (float)dist;
                    anchor.rejectStreak = 0;
                }

                anchor.rawDist = valid ? (float)dist : anchor.rawDist;
                float filtered = valid ? anchor.kalman.update((float)dist, UWB_KALMAN_Q, UWB_KALMAN_R) : anchor.kalman.x;
                ESP_LOGD(TAG, "✅ [%s/%s] raw=%d filtered=%.1f cm  latency=%ums", anchor.position, anchor.moduleName, dist, filtered, latency);
                if (&anchor == &anchorLeft) {
                    _uwbData.distLeft  = filtered; _uwbData.validLeft  = valid;
                } else {
                    _uwbData.distRight = filtered; _uwbData.validRight = valid;
                }
                anchor.pollState = PollState::IDLE;
                return;
            }
            anchor.bufPos = 0;
        } else if (c != '\r') {
            anchor.buf[anchor.bufPos++] = c;
        }
    }

    if (millis() - anchor.sentAt >= UWB_RESPONSE_TIMEOUT_MS) {
        ESP_LOGW(TAG, "⚠️ [%s/%s] no response  latency=%ums", anchor.position, anchor.moduleName, millis() - anchor.sentAt);
        if (&anchor == &anchorLeft) {
            _uwbData.distLeft = -1; _uwbData.validLeft = false;
        } else {
            _uwbData.distRight = -1; _uwbData.validRight = false;
        }
        anchor.pollState = PollState::IDLE;
    }
}

void uwb_update() {
    static uint32_t  lastCycleEnd  = 0;
    static uint32_t  lastUartCheck = 0;
    static CyclePhase phase        = CyclePhase::WAIT;

    if (phase == CyclePhase::WAIT) {
        if (millis() - lastCycleEnd < UWB_POLL_INTERVAL_MS) return;
        uwb_start_poll(anchorLeft);
        phase = CyclePhase::LEFT_PENDING;
        return;
    }

    // Rate-limit UART polling to 1kHz
    if (millis() - lastUartCheck < 1) return;
    lastUartCheck = millis();

    if (phase == CyclePhase::LEFT_PENDING) {
        uwb_update_anchor(anchorLeft);
        if (anchorLeft.pollState == PollState::IDLE) {
            uwb_start_poll(anchorRight);
            phase = CyclePhase::RIGHT_PENDING;
        }
        return;
    }

    // phase == RIGHT_PENDING
    uwb_update_anchor(anchorRight);
    if (anchorRight.pollState == PollState::IDLE) {
        bool lv = anchorLeft.rawDist  >= 0;
        bool rv = anchorRight.rawDist >= 0;
        if (lv && rv)  _uwbData.distFast = (anchorLeft.rawDist + anchorRight.rawDist) / 2.0f;
        else if (lv)   _uwbData.distFast = anchorLeft.rawDist;
        else if (rv)   _uwbData.distFast = anchorRight.rawDist;
        _uwbData.timestamp = millis();
        lastCycleEnd    = millis();
        phase = CyclePhase::WAIT;
    }
}

const UWBReading& uwb_get() {
    return _uwbData;
}

// =============================================================================
// UWB serial passthrough — remove this block (and the call in main.cpp) to disable
// Usage via telnet/serial: uwb left|right|rear <AT command>
// Example: uwb left AT+ADDRESS?
// =============================================================================
static char _passthroughBuf[128] = {};
static int  _passthroughPos      = 0;

static UWBAnchor* uwb_anchor_by_name(const char* name) {
    if (strcmp(name, "left")  == 0) return &anchorLeft;
    if (strcmp(name, "right") == 0) return &anchorRight;
    if (strcmp(name, "front") == 0) return &anchorFront;
    return nullptr;
}

void uwb_passthrough_update() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (_passthroughPos == 0) continue;
            _passthroughBuf[_passthroughPos] = '\0';
            _passthroughPos = 0;

            if (strncmp(_passthroughBuf, "uwb ", 4) != 0) continue;
            char* rest  = _passthroughBuf + 4;
            char* space = strchr(rest, ' ');
            if (!space) { Serial.println("usage: uwb left|right|rear <AT command>"); continue; }
            *space = '\0';
            char* cmd = space + 1;

            UWBAnchor* anchor = uwb_anchor_by_name(rest);
            if (!anchor) { Serial.printf("unknown anchor: %s\n", rest); continue; }

            while (anchor->serial.available()) anchor->serial.read();
            anchor->serial.print(cmd);
            anchor->serial.print("\r\n");

            uint32_t sentAt = millis();
            char buf[128]; int pos = 0; bool gotResponse = false;
            while (millis() - sentAt < UWB_RESPONSE_TIMEOUT_MS) {
                while (anchor->serial.available() && pos < 127) {
                    char rc = anchor->serial.read();
                    if (rc == '\n') {
                        buf[pos] = '\0';
                        if (pos > 0) { Serial.println(buf); gotResponse = true; }
                        pos = 0;
                    } else if (rc != '\r') {
                        buf[pos++] = rc;
                    }
                }
            }
            if (!gotResponse) Serial.println("(no response)");
        } else if (_passthroughPos < 127) {
            _passthroughBuf[_passthroughPos++] = c;
        }
    }
}
// =============================================================================
// End UWB serial passthrough
// =============================================================================
