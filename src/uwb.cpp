// UWB ranging driver for RYUW122 anchors. All communication uses raw AT commands.
// Per-anchor Kalman filtering and outlier rejection; exposes distLeft/distRight (filtered) and distFast (raw avg).

#include "uwb.h"
#include "config.h"
#include "utils.h"
#include "wifi_config.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "uwb";

static HardwareSerial uartFront(0);

enum class PollState  { IDLE, AWAITING };
enum class CyclePhase { WAIT, LEFT_PENDING, RIGHT_PENDING, FRONT_PENDING };

struct UWBAnchor {
    const char*     position;
    char            moduleName[16] = {};
    HardwareSerial& serial;
    int             pinTx;
    int             pinRx;
    PollState    pollState    = PollState::IDLE;
    uint32_t     sentAt       = 0;
    char         buf[128]     = {};
    int          bufPos       = 0;
    KalmanFilter kalman       = {};
    float        rawDist      = -1.0f;
    float        prevDist     = -1.0f; // last accepted distance, -1 = no baseline yet
    int          rejectStreak = 0;     // consecutive outlier rejections
};

static UWBReading _uwbData = {};

static UWBAnchor anchorLeft  { "left",  {}, Serial1,   PIN_UWB_LEFT_TX,  PIN_UWB_LEFT_RX  };
static UWBAnchor anchorRight { "right", {}, Serial2,   PIN_UWB_RIGHT_TX, PIN_UWB_RIGHT_RX };
static UWBAnchor anchorFront { "front", {}, uartFront, PIN_UWB_FRONT_TX, PIN_UWB_FRONT_RX };

// --- AT command helpers (blocking, init/cal use only) ------------------------

// Sends a query command and copies the value that follows prefix into out.
static bool uwb_at_query(UWBAnchor& anchor, const char* cmd, const char* prefix, char* out, size_t outLen) {
    while (anchor.serial.available()) anchor.serial.read();
    anchor.serial.print(cmd); anchor.serial.print("\r\n");
    uint32_t sentAt = millis();
    char buf[64]; int pos = 0; buf[0] = '\0';
    while (millis() - sentAt < UWB_RESPONSE_TIMEOUT_MS) {
        while (anchor.serial.available() && pos < 63) {
            char c = anchor.serial.read();
            if (c == '\n') {
                buf[pos] = '\0';
                const char* p = strstr(buf, prefix);
                if (p) {
                    strncpy(out, p + strlen(prefix), outLen - 1);
                    out[outLen - 1] = '\0';
                    return true;
                }
                pos = 0;
            } else if (c != '\r') buf[pos++] = c;
        }
    }
    return false;
}

// Sends a command and returns true if any response line contains expected.
static bool uwb_at_send(UWBAnchor& anchor, const char* cmd, const char* expected) {
    char discard[2];
    return uwb_at_query(anchor, cmd, expected, discard, sizeof(discard));
}

// --- Calibration -------------------------------------------------------------

static int uwb_get_cal(UWBAnchor& anchor) {
    char val[16] = {};
    if (!uwb_at_query(anchor, "AT+CAL?", "+CAL=", val, sizeof(val))) return 0;
    return atoi(val);
}

static bool uwb_set_cal(UWBAnchor& anchor, int cal) {
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "AT+CAL=%d", cal);
    return uwb_at_send(anchor, cmd, "+OK");
}

// --- Init / diagnostics ------------------------------------------------------

// Parses a +ANCHOR_RCV= response line and returns the distance field in cm, or -1.
static int parse_anchor_rcv(const char* buf) {
    const char* p = strstr(buf, "+ANCHOR_RCV=");
    if (!p) return -1;
    p += strlen("+ANCHOR_RCV=");
    for (int i = 0; i < 3 && p; i++) p = strchr(p, ',') + 1;
    return p ? atoi(p) : -1;
}

// Blocking single poll — used only at init time for calibration/diagnostics.
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

static bool uwb_init_anchor(UWBAnchor& anchor) {
    anchor.serial.begin(115200, SERIAL_8N1, anchor.pinRx, anchor.pinTx);

    // Reset after serial is ready, then drain until 200ms of idle (matches original library sequence)
    digitalWrite(PIN_UWB_NRST, LOW);  delay(5);
    digitalWrite(PIN_UWB_NRST, HIGH);
    uint32_t idleStart = millis();
    while (millis() - idleStart < 200) {
        if (anchor.serial.available()) { anchor.serial.read(); idleStart = millis(); }
    }

    uwb_at_query(anchor, "AT+ADDRESS?", "+ADDRESS=", anchor.moduleName, sizeof(anchor.moduleName));

    bool ok = uwb_at_send(anchor, "AT", "+OK");
    if (ok) {
        ESP_LOGI(TAG, "✅ [%s/%s] ready", anchor.position, anchor.moduleName);
    } else {
        ESP_LOGE(TAG, "❌ [%s/%s] not responding", anchor.position, anchor.moduleName);
    }
    return ok;
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

static void uwb_calibrate_anchor(UWBAnchor& anchor, float knownDistCm, int numSamples) {
    if (uwb_poll_blocking(anchor) < 0) {
        ESP_LOGW(TAG, "⚠️ [%s/%s] calibration skipped — tag not found", anchor.position, anchor.moduleName);
        return;
    }

    int existingCal = uwb_get_cal(anchor);
    int samples[UWB_CALIBRATION_SAMPLES];
    int count = 0;

    for (int i = 0; i < numSamples && count < UWB_CALIBRATION_SAMPLES; i++) {
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

    uwb_set_cal(anchor, newCal);
}

void uwb_init() {
    pinMode(PIN_UWB_NRST, OUTPUT);

    uwb_init_anchor(anchorLeft);
    uwb_init_anchor(anchorRight);
    uwb_init_anchor(anchorFront);
    if (UWB_CALIBRATE_ON_STARTUP) {
        // Tag is assumed on the car centerline at UWB_CALIBRATION_DISTANCE_CM forward of the left/right anchor line.
        // Front anchor's known distance is computed from its offset position to that tag location.
        float frontCalDist = sqrtf(UWB_FRONT_X_CM * UWB_FRONT_X_CM +
                                   (UWB_CALIBRATION_DISTANCE_CM - UWB_FRONT_Y_CM) *
                                   (UWB_CALIBRATION_DISTANCE_CM - UWB_FRONT_Y_CM));
        uwb_calibrate_anchor(anchorLeft,  UWB_CALIBRATION_DISTANCE_CM, UWB_CALIBRATION_SAMPLES);
        uwb_calibrate_anchor(anchorRight, UWB_CALIBRATION_DISTANCE_CM, UWB_CALIBRATION_SAMPLES);
        uwb_calibrate_anchor(anchorFront, frontCalDist,                 UWB_CALIBRATION_SAMPLES);
    }
}

// Writes a ranging result into _uwbData for whichever anchor this is. Negative = invalid.
static void uwb_write_dist(UWBAnchor& anchor, float dist) {
    if      (&anchor == &anchorLeft)  _uwbData.distLeft  = dist;
    else if (&anchor == &anchorRight) _uwbData.distRight = dist;
    else                              _uwbData.distFront = dist;
}

// --- Non-blocking ranging loop -----------------------------------------------

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
                uwb_write_dist(anchor, valid ? filtered : -1.0f);
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
        uwb_write_dist(anchor, -1.0f);
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

    if (phase == CyclePhase::RIGHT_PENDING) {
        uwb_update_anchor(anchorRight);
        if (anchorRight.pollState == PollState::IDLE) {
            uwb_start_poll(anchorFront);
            phase = CyclePhase::FRONT_PENDING;
        }
        return;
    }

    // phase == FRONT_PENDING
    uwb_update_anchor(anchorFront);
    if (anchorFront.pollState == PollState::IDLE) {
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
