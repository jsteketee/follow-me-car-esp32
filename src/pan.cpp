// Pan servo driver for the UWB anchor mount. Owns the µs↔degree map and slewed
// (rate-limited) moves so the commanded angle never runs far ahead of the physical
// one. The calibration routine at the bottom (PAN_CAL_TEST builds only) measures
// the map experimentally using the UWB's own bearing readings.
//
// Drives the pin with the core LEDC API directly, NOT ESP32Servo: on the ESP32-S3
// that library puts 3rd+ servos on MCPWM timer 1 but routes their GPIO to the
// timer-0 output signal (ESP32PWM.cpp:492 picks the io signal by operator only),
// so the pin silently carries another servo's waveform. Verified 2026-07-13 with
// the servo-test sweep: direct LEDC drives these pins fine.
#include "pan.h"
#include "config.h"
#include "uwb.h"
#include "utils.h"
#include <Arduino.h>

static const char* TAG = "pan";

// LEDC config: 50Hz servo frame, 14-bit duty (S3 LEDC max; ~1.2µs per count).
static const uint32_t PAN_LEDC_FREQ_HZ   = 50;
static const uint8_t  PAN_LEDC_RES_BITS  = 14;
static const uint32_t PAN_LEDC_PERIOD_US = 1000000UL / PAN_LEDC_FREQ_HZ;

static bool _panAttached = false;
static int  _panUs = PAN_SERVO_CENTER_US + PAN_SERVO_TRIM_US;  // last commanded pulse width

// µs of travel per 20ms tick that realizes PAN_SLEW_DEG_PER_S; floored at 1 so a
// pending move always progresses.
static int pan_slew_step_us() {
    int step = (int)lroundf(PAN_SLEW_DEG_PER_S * 0.02f * fabsf(PAN_SERVO_US_PER_DEG));
    return step < 1 ? 1 : step;
}

// Writes a pulse width to the pin (clamped to the configured endpoints) and records it.
static void pan_write_us(int us) {
    _panUs = constrain(us, PAN_SERVO_MIN_US, PAN_SERVO_MAX_US);
    if (!_panAttached) return;
    uint32_t duty = (uint64_t)_panUs * ((1UL << PAN_LEDC_RES_BITS) - 1) / PAN_LEDC_PERIOD_US;
    ledcWrite(PIN_SERVO_UWB, duty);
}

// Trimmed center pulse width — the µs value for anchor boresight = car forward.
static int pan_center_us() { return PAN_SERVO_CENTER_US + PAN_SERVO_TRIM_US; }

// Converts a pulse width to the anchor-frame pan angle (0 = forward, + = right).
static float pan_us_to_deg(int us) {
    return (us - pan_center_us()) / PAN_SERVO_US_PER_DEG;
}

// Current commanded pan angle in degrees.
float pan_get_angle() { return pan_us_to_deg(_panUs); }

static RateGate _panTrackGate{ 20 };

// Slews toward the Pi-commanded target (from CommandData, routed by main's loop) at
// PAN_SLEW_DEG_PER_S, so a large commanded jump becomes slow consistent motion and
// pan_get_angle stays honest throughout. The target is clamped to the symmetric
// ±PAN_MAX_DEG contract — one limit both ways even though one side of the mount can
// physically travel farther; the µs endpoint clamp backstops it.
void pan_update(float targetDeg) {
    float dt;
    if (!_panTrackGate.tick(dt)) return;
    if (!_panAttached) return;
    targetDeg = constrain(targetDeg, -PAN_MAX_DEG, PAN_MAX_DEG);
    int targetUs = constrain(pan_center_us() + (int)lroundf(targetDeg * PAN_SERVO_US_PER_DEG),
                             PAN_SERVO_MIN_US, PAN_SERVO_MAX_US);
    if (_panUs == targetUs) return;
    int maxStep = pan_slew_step_us();
    int step = constrain(targetUs - _panUs, -maxStep, maxStep);
    pan_write_us(_panUs + step);
}

// Attaches the pan servo pin to LEDC and moves it to trimmed center.
void pan_init() {
    _panAttached = ledcAttach(PIN_SERVO_UWB, PAN_LEDC_FREQ_HZ, PAN_LEDC_RES_BITS);
    if (!_panAttached) {
        Serial.printf("[%s] ❌ ledcAttach FAILED on pin %d\n", TAG, PIN_SERVO_UWB);
        return;
    }
    pan_write_us(pan_center_us());
    Serial.printf("[%s] ✅ pan servo (pin %d, LEDC) ready — center %d µs\n",
                  TAG, PIN_SERVO_UWB, _panUs);
}

#ifdef PAN_CAL_TEST
// =============================================================================
// Calibration (bench only — pan-cal env). Everything below is deletable once the
// config values are locked in; nothing outside this block depends on it.
// =============================================================================

// Cal tunables — local to the routine, not config: they die with this code.
static const int      CAL_POSITIONS        = 5;     // measurement points, evenly spaced across ±50% of travel
static const uint32_t CAL_MEASURE_MS       = 5000;  // sampling window per position
static const int      CAL_MIN_SAMPLES      = 25;    // fewer frames than this in the window = UWB too flaky, abort
static const int      CAL_MAX_SAMPLES      = 350;   // frame buffer bound (~50Hz × 5s + headroom)
static const int      CAL_DISCARD_FRAMES   = 3;     // fresh frames dropped after each move, beyond the settle wait
static const uint32_t CAL_SETTLE_MS        = 1500;  // buffer after each move before sampling starts
static const uint32_t CAL_FRAME_TIMEOUT_MS = 3000;  // max gap between UWB frames before aborting
static const uint32_t CAL_SLEW_TICK_MS     = 20;    // slew step period (matches 50Hz servo frame)

// Waits while pumping the UWB parser so the UART buffer never overflows mid-cal.
static void cal_wait_ms(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        uwb_update();
        delay(5);
    }
}

// Slews the servo to targetUs at PAN_SLEW_DEG_PER_S, pumping the
// UWB parser each tick. Slow on purpose: a bind at an endpoint is visible/audible.
static void cal_slew_to(int targetUs) {
    targetUs = constrain(targetUs, PAN_SERVO_MIN_US, PAN_SERVO_MAX_US);
    Serial.printf("[%s] slew %d → %d µs\n", TAG, _panUs, targetUs);
    while (_panUs != targetUs) {
        int maxStep = pan_slew_step_us();
        int step = constrain(targetUs - _panUs, -maxStep, maxStep);
        pan_write_us(_panUs + step);
        cal_wait_ms(CAL_SLEW_TICK_MS);
    }
}

// Median of the first n floats in buf (sorts in place; n is small).
static float cal_median(float* buf, int n) {
    for (int i = 1; i < n; i++) {
        float v = buf[i];
        int j = i - 1;
        while (j >= 0 && buf[j] > v) { buf[j + 1] = buf[j]; j--; }
        buf[j + 1] = v;
    }
    return (n % 2) ? buf[n / 2] : 0.5f * (buf[n / 2 - 1] + buf[n / 2]);
}

// Collects UWB frames for CAL_MEASURE_MS at the current pan position and reports the
// bearing median (plus spread and distance for the log). False if the UWB times out
// mid-window or delivers too few frames to trust.
static bool cal_measure(const char* label, float* bearingMedOut) {
    static float bearings[CAL_MAX_SAMPLES];  // static: keeps ~3KB off the stack
    static float dists[CAL_MAX_SAMPLES];

    // Settle the servo and the anchor's tracking, then drop the first few frames.
    cal_wait_ms(CAL_SETTLE_MS);
    int discarded = 0, collected = 0;
    uint32_t windowStart = millis();
    uint32_t lastFrameMs = millis();
    while (millis() - windowStart < CAL_MEASURE_MS) {
        if (uwb_update()) {
            lastFrameMs = millis();
            if (discarded < CAL_DISCARD_FRAMES) { discarded++; continue; }
            if (collected < CAL_MAX_SAMPLES) {
                const UWBReading& r = uwb_get();
                bearings[collected] = r.angleDeg;
                dists[collected]    = r.distCm;
                collected++;
            }
        }
        if (millis() - lastFrameMs > CAL_FRAME_TIMEOUT_MS) {
            Serial.printf("[%s] ❌ %s: no UWB frame in %lums (%d collected) — is the tag on?\n",
                          TAG, label, (unsigned long)CAL_FRAME_TIMEOUT_MS, collected);
            return false;
        }
        delay(2);
    }
    if (collected < CAL_MIN_SAMPLES) {
        Serial.printf("[%s] ❌ %s: only %d frames in %lums (need ≥%d) — UWB too flaky for calibration\n",
                      TAG, label, collected, (unsigned long)CAL_MEASURE_MS, CAL_MIN_SAMPLES);
        return false;
    }

    float bearingMin = bearings[0], bearingMax = bearings[0], bearingSum = 0.0f;
    for (int i = 0; i < collected; i++) {
        if (bearings[i] < bearingMin) bearingMin = bearings[i];
        if (bearings[i] > bearingMax) bearingMax = bearings[i];
        bearingSum += bearings[i];
    }
    float bearingMed = cal_median(bearings, collected);  // sorts; min/max/sum taken first
    float distMed    = cal_median(dists,    collected);

    Serial.printf("[%s] 📐 %s @ %dµs: bearing med=%.2f°  mean=%.2f°  spread=[%.1f, %.1f]  dist=%.0fcm  (n=%d)\n",
                  TAG, label, _panUs, bearingMed, bearingSum / collected,
                  bearingMin, bearingMax, distMed, collected);
    *bearingMedOut = bearingMed;
    return true;
}

// Full calibration sequence: mechanical sweep (center → min → max → center), then
// bearing measurements at CAL_POSITIONS points spanning ±30% of travel, then a
// least-squares fit of bearing vs pulse width for the µs↔degree map and trim.
void pan_calibrate() {
    const int centerUs = pan_center_us();

    // Measurement points: evenly spaced across ±30% of each side's travel, visited
    // monotonically min→max so every position is approached from the same direction
    // (consistent gear backlash instead of mixed). ±30% keeps apparent bearings
    // inside ~±25° — DW3000 AOA response goes nonlinear past ~±40°, which bends
    // the fit if measurement points reach it.
    int   posUs[CAL_POSITIONS];
    float posBear[CAL_POSITIONS];
    for (int i = 0; i < CAL_POSITIONS; i++) {
        float frac = -0.3f + 0.6f * i / (CAL_POSITIONS - 1);  // −0.3 … +0.3
        int span = (frac < 0) ? (centerUs - PAN_SERVO_MIN_US) : (PAN_SERVO_MAX_US - centerUs);
        posUs[i] = centerUs + (int)(frac * span);
    }

    Serial.printf("\n[%s] ===== PAN CALIBRATION =====\n", TAG);
    Serial.printf("[%s] endpoints %d–%d µs, center %d µs (trim %+d), %d points × %lums\n",
                  TAG, PAN_SERVO_MIN_US, PAN_SERVO_MAX_US, centerUs, PAN_SERVO_TRIM_US,
                  CAL_POSITIONS, (unsigned long)CAL_MEASURE_MS);
    Serial.printf("[%s] tag should be stationary, roughly dead ahead, 2–4m away\n", TAG);

    // --- Phase 1: mechanical sweep — watch/listen for binding at the endpoints.
    Serial.printf("[%s] --- phase 1: mechanical sweep ---\n", TAG);
    cal_slew_to(centerUs);
    cal_slew_to(PAN_SERVO_MIN_US);
    cal_slew_to(PAN_SERVO_MAX_US);
    cal_slew_to(centerUs);
    Serial.printf("[%s] sweep done\n", TAG);

    // --- Phase 2: bearing measurement at each position, min→max.
    Serial.printf("[%s] --- phase 2: bearing measurements ---\n", TAG);
    bool ok = true;
    for (int i = 0; i < CAL_POSITIONS && ok; i++) {
        char label[16];
        snprintf(label, sizeof(label), "point %d/%d", i + 1, CAL_POSITIONS);
        cal_slew_to(posUs[i]);
        ok = cal_measure(label, &posBear[i]);
    }
    cal_slew_to(centerUs);
    if (!ok) {
        Serial.printf("[%s] ❌ calibration aborted — servo returned to center\n", TAG);
        return;
    }

    // --- Phase 3: least-squares fit of bearing (y) on pulse width (x).
    // Rotating the anchor by +θ shifts the apparent bearing by −θ, so the map is
    // us_per_deg = −1/slope, and the trim is the µs where the fit crosses bearing 0
    // (valid as trim only if the tag really was dead ahead).
    Serial.printf("[%s] --- phase 3: fit ---\n", TAG);
    float meanX = 0, meanY = 0;
    for (int i = 0; i < CAL_POSITIONS; i++) { meanX += posUs[i]; meanY += posBear[i]; }
    meanX /= CAL_POSITIONS;  meanY /= CAL_POSITIONS;
    float sxx = 0, sxy = 0;
    for (int i = 0; i < CAL_POSITIONS; i++) {
        sxx += (posUs[i] - meanX) * (posUs[i] - meanX);
        sxy += (posUs[i] - meanX) * (posBear[i] - meanY);
    }
    float slope = sxy / sxx;                    // deg of bearing per µs
    float intercept = meanY - slope * meanX;    // bearing at 0µs (extrapolated)

    // Guard: a near-zero slope means the anchor isn't rotating (or the UWB isn't
    // tracking) — 3° across the full measured span is the floor for a usable fit.
    float spanUs = posUs[CAL_POSITIONS - 1] - posUs[0];
    if (fabsf(slope) * spanUs < 3.0f) {
        Serial.printf("[%s] ⚠️ bearing moved <3° across the whole sweep — fit unreliable, check mount/tag\n", TAG);
        return;
    }

    // Per-point residuals: how far each measurement sits off the fitted line.
    float maxResid = 0, sumSqResid = 0;
    for (int i = 0; i < CAL_POSITIONS; i++) {
        float resid = posBear[i] - (intercept + slope * posUs[i]);
        sumSqResid += resid * resid;
        if (fabsf(resid) > maxResid) maxResid = fabsf(resid);
        Serial.printf("[%s]   %dµs: bearing %+.2f°  residual %+.2f°\n", TAG, posUs[i], posBear[i], resid);
    }
    float rmsResid = sqrtf(sumSqResid / CAL_POSITIONS);
    Serial.printf("[%s] fit: slope %+.4f°/µs  residuals rms=%.2f° max=%.2f°%s\n",
                  TAG, slope, rmsResid, maxResid,
                  maxResid > 3.0f ? " ⚠️ — poor linearity or a disturbed measurement, consider rerunning" : "");

    float usPerDeg  = -1.0f / slope;                       // anchor-frame µs per degree (signed)
    float zeroUs    = -intercept / slope;                  // µs where bearing crosses 0
    int   suggTrim  = (int)lroundf(zeroUs - PAN_SERVO_CENTER_US);

    Serial.printf("[%s] ===== suggested config =====\n", TAG);
    Serial.printf("[%s] #define PAN_SERVO_US_PER_DEG   %.2ff\n", TAG, usPerDeg);
    Serial.printf("[%s] #define PAN_SERVO_TRIM_US      %+d   // ONLY if the tag was placed dead ahead (bearing-zero at %.0fµs)\n",
                  TAG, suggTrim, zeroUs);
    Serial.printf("[%s] ===== calibration complete =====\n\n", TAG);
}
#endif  // PAN_CAL_TEST
