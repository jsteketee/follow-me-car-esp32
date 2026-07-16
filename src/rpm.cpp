// RPM driver: hall-effect interrupt for speed and odometry; AS5600 encoder for cogging detection.
// Hall-effect is the source of truth for speed and odometry. The encoder's high-resolution
// angle data detects motor cogging — oscillation at a reluctance point without net forward motion.
#include "rpm.h"
#include "config.h"
#include "runtime_config.h"
#include "utils.h"
#include "esp_log.h"
#include <Arduino.h>
#include <Wire.h>

static const char* TAG = "rpm";

// =============================================================================
// Hall-effect sensor (speed + odometry)
// =============================================================================
static volatile uint32_t _hallLastPulseUs = 0;
static volatile uint32_t _hallPeriodUs    = 0;
static volatile uint32_t _hallPulseCount  = 0;
static volatile bool     _hallPulseValid  = false;
static float             _hallEmaSpeed    = 0.0f;
static int               _hallSpikeStreak = 0;
static uint32_t          _hallLastCountTracked = 0; // pulse count snapshot for odometry
static HzTracker         _hallHz;                   // pulse arrival rate — 0 at standstill, scales with wheel speed

// ISR — records inter-pulse period and increments pulse counter.
static void IRAM_ATTR on_hall_pulse() {
    uint32_t now = micros();
    if (_hallPulseValid) {
        _hallPeriodUs = now - _hallLastPulseUs;
    }
    _hallLastPulseUs = now;
    _hallPulseValid  = true;
    _hallPulseCount++;
}

// =============================================================================
// AS5600 encoder (cogging detection)
// =============================================================================
static const uint8_t REG_ANGLE     = 0x0E; // filtered 12-bit angle, high byte at 0x0E low at 0x0F
static const uint8_t REG_RAW_ANGLE = 0x0C; // unfiltered 12-bit angle, high byte at 0x0C low at 0x0D
static const uint8_t REG_STATUS    = 0x0B; // magnet status: bit5=MD(detected), bit4=ML(weak), bit3=MH(strong)
static const uint8_t REG_AGC       = 0x1A; // automatic gain control, 0-128 at 3.3V supply
static const uint8_t REG_MAGNITUDE = 0x1B; // CORDIC magnitude, 12-bit, high byte at 0x1B low at 0x1C

static int      _encLastAngle      = -1;    // -1 = not yet seeded
static float    _encEmaVelocityMph = 0.0f;  // EMA of signed encoder velocity (forward = positive)
static AngleKalman  _fused2Kf;              // 2-state fused speed [speed, accelBias] — same struct as the heading filter, speed domain
                                            // (retired 1-D encoder KF + 1-D fusion archived in test/rpm_testing.cpp)
static RateGate     _fused2PredictGate{ 20 };  // predict cadence matched to the 50 Hz linear-accel report

// Speed-ramped encoder R for the fused filter; NAN past the ramp end = skip the update
// (aliased readings are biased, not noisy). Gates on the caller's fused estimate, never
// the encoder's own reading — an aliased encoder reads slow and would vote itself back in.
static float fused_enc_r(float estimateMph) {
    // Sliders can momentarily put end <= start; degrade to a hard cutoff at start.
    float width = rtConfig.fusedRampEndMph - rtConfig.fusedRampStartMph;
    if (width <= 0.0f)
        return fabsf(estimateMph) < rtConfig.fusedRampStartMph ? rtConfig.fusedEncR : NAN;
    float t = (fabsf(estimateMph) - rtConfig.fusedRampStartMph) / width;
    if (t >= 1.0f) return NAN;
    if (t < 0.0f) t = 0.0f;
    return rtConfig.fusedEncR * powf(10.0f, RPM_FUSED_ENC_R_DECADES * t);
}
static RateGate _encPollGate{ RPM_POLL_INTERVAL_MS };

// Circular buffer of recent encoder deltas for cogging analysis.
static int      _coggingDeltas[RPM_COGGING_WINDOW] = {};
static int      _coggingIdx    = 0;
static bool     _coggingBufFull = false;

// Latching cogging state machine.
enum class CogState { CLEAR, COGGING };
static CogState  _cogState        = CogState::CLEAR;
static uint32_t  _cogClearStartMs = 0;  // wall time when sustained clear velocity began

// =============================================================================
// Output
// =============================================================================
static RPMData _rpmData = {};

// =============================================================================
// Private helpers — encoder
// =============================================================================

// Reads the AS5600 filtered 12-bit angle over I2C; returns -1 on error.
static int read_encoder_angle() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_ANGLE);
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return -1;
    return ((Wire.read() & 0x0F) << 8) | Wire.read();
}

// Reads STATUS register and logs a warning if the magnet is missing or out of range.
static void check_magnet_status() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_STATUS);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(AS5600_ADDR, (uint8_t)1) != 1) {
        ESP_LOGW(TAG, "⚠️ AS5600 status read failed");
        return;
    }
    uint8_t s  = Wire.read();
    bool    md = s & (1 << 5);
    bool    ml = s & (1 << 4);
    bool    mh = s & (1 << 3);

    if      (!md) ESP_LOGW(TAG, "⚠️ AS5600 no magnet detected");
    else if (ml)  ESP_LOGW(TAG, "⚠️ AS5600 magnet too weak — move magnet closer");
    else if (mh)  ESP_LOGW(TAG, "⚠️ AS5600 magnet too strong — move magnet farther");
}

// Logs a MON line (status flags, AGC, magnitude, raw angle) at a fixed 0.25s rate, for bench monitoring.
static void log_monitor() {
    static uint32_t lastMonMs = 0;
    uint32_t now = millis();
    if (now - lastMonMs < 250) return;
    lastMonMs = now;

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_STATUS);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(AS5600_ADDR, (uint8_t)1) != 1) {
        ESP_LOGW(TAG, "MON: status read failed");
        return;
    }
    uint8_t s  = Wire.read();
    bool    md = s & (1 << 5);
    bool    ml = s & (1 << 4);
    bool    mh = s & (1 << 3);

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_AGC);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, (uint8_t)1);
    int agc = Wire.read();

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_MAGNITUDE);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
    int mag = ((Wire.read() & 0x0F) << 8) | Wire.read();

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_RAW_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
    int raw = ((Wire.read() & 0x0F) << 8) | Wire.read();

    ESP_LOGI(TAG, "MON: MD=%d ML=%d MH=%d  AGC=%d  MAG=%d  RAW=%d (%.2f deg)",
             md, ml, mh, agc, mag, raw, raw * (360.0f / 4096.0f));
}

// Analyzes the cogging delta buffer; outputs sign change count and net displacement.
// Returns true if sign changes and net displacement both meet cogging thresholds.
static bool analyze_cogging(int& outSignChanges, int& outNetDelta) {
    int count = _coggingBufFull ? RPM_COGGING_WINDOW : _coggingIdx;
    outSignChanges = 0;
    outNetDelta    = 0;
    if (count < 2) return false;

    for (int i = 0; i < count; i++) {
        outNetDelta += _coggingDeltas[i];
        if (i > 0 && _coggingDeltas[i] != 0 && _coggingDeltas[i - 1] != 0) {
            if ((_coggingDeltas[i] > 0) != (_coggingDeltas[i - 1] > 0)) {
                outSignChanges++;
            }
        }
    }
    return (outSignChanges >= RPM_COGGING_MIN_SIGN_CHANGES)
        && (abs(outNetDelta) <= RPM_COGGING_MAX_NET_COUNTS);
}

// Polls the encoder and updates the cogging flag. 
static bool update_encoder() {
    float dt;
    if (!_encPollGate.tick(dt)) return false;

    int angle = read_encoder_angle();
    if (angle < 0) {
        ESP_LOGW(TAG, "⚠️ AS5600 read failed");
        return false;
    }

    if (_encLastAngle < 0) {
        _encLastAngle = angle;
        return true;
    }

    // Shortest-path delta, handling the 0/4095 wraparound boundary.
    int delta = angle - _encLastAngle;
    if (delta >  2048) delta -= AS5600_COUNTS_PER_REV;
    if (delta < -2048) delta += AS5600_COUNTS_PER_REV;
    _encLastAngle = angle;

    // EMA encoder velocity in mph — forward positive, backward negative. Divides by
    // the gate's measured dt, not the nominal interval: loop stalls (IMU drains) can
    // delay a poll, and nominal-dt would read those samples as faster than reality.
    float rawVelMph = ((float)(-delta) * RPM_CM_PER_COUNT / dt) * 0.0223694f;
    _encEmaVelocityMph += RPM_COGGING_ENC_EMA_ALPHA * (rawVelMph - _encEmaVelocityMph);
    _rpmData.encRawMph   = rawVelMph;
    _rpmData.encSpeedMph = _encEmaVelocityMph;

    // Fused-speed encoder correction (250 Hz), soft-gated by the speed-ramped R.
    float fused2EncR = fused_enc_r(_fused2Kf.angle);
    if (!isnan(fused2EncR)) {
        _fused2Kf.correct(rawVelMph, fused2EncR);
        _rpmData.fusedSpeedMph = _fused2Kf.angle;
    }

    // Push delta into circular buffer.
    _coggingDeltas[_coggingIdx] = delta;
    _coggingIdx = (_coggingIdx + 1) % RPM_COGGING_WINDOW;
    if (_coggingIdx == 0) _coggingBufFull = true;

    // Cogging detection only meaningful at low speed — above threshold, false positives dominate.
    if (_rpmData.hallSpeedMph >= RPM_COGGING_MAX_SPEED_MPH) {
        if (_rpmData.cogging) {
            ESP_LOGI(TAG, "🟢 cogging cleared (speed above threshold)");
            _rpmData.cogging  = false;
            _cogState         = CogState::CLEAR;
            _cogClearStartMs  = 0;
        }
        return true;
    }

    int signChanges = 0, netDelta = 0;
    bool rawCogging = analyze_cogging(signChanges, netDelta);
    _rpmData.signChanges = signChanges;
    uint32_t now    = millis();

    // Diagnostic log at 5 Hz — shows algorithm inputs and result.
    static uint32_t lastDiagMs = 0;
    if (now - lastDiagMs >= 200) {
        lastDiagMs = now;
        const char* stateStr = (_cogState == CogState::COGGING) ? "COGGING" : "CLEAR";
        ESP_LOGI(TAG, "cog diag: state=%s  raw=%d  sc=%d/%d  net=%d/%d  enc=%.3fmph  hall=%.3fmph",
            stateStr, (int)rawCogging,
            signChanges, RPM_COGGING_MIN_SIGN_CHANGES,
            abs(netDelta), RPM_COGGING_MAX_NET_COUNTS,
            _encEmaVelocityMph, _rpmData.hallSpeedMph);
    }

    if (_cogState == CogState::CLEAR) {
        // Enter cogging: sign-change pattern detected.
        if (rawCogging) {
            _cogState        = CogState::COGGING;
            _cogClearStartMs = 0;
            _rpmData.cogging = true;
            ESP_LOGI(TAG, "🔴 cogging detected (vel=%.3fmph sc=%d)", _encEmaVelocityMph, signChanges);
        }
    } else { // CogState::COGGING
        if (_encEmaVelocityMph > RPM_COGGING_ENC_CLEAR_MPH) {
            // Sustained positive (forward) velocity — start or continue exit timer.
            if (_cogClearStartMs == 0) _cogClearStartMs = now;
            if ((now - _cogClearStartMs) >= (uint32_t)(2000.0f / RPM_COGGING_FREQ_HZ)) {
                _cogState        = CogState::CLEAR;
                _rpmData.cogging = false;
                _cogClearStartMs = 0;
                ESP_LOGI(TAG, "🟢 cogging cleared (vel=%.3fmph for %.0fms)", _encEmaVelocityMph, 2000.0f / RPM_COGGING_FREQ_HZ);
            }
        } else {
            // Velocity not sustained — reset exit timer; must be continuous to clear.
            _cogClearStartMs = 0;
        }
    }
    return true;
}

// =============================================================================
// Private helpers — hall-effect
// =============================================================================

// Reads latest hall-effect ISR state, updates EMA speed, and accumulates odometry. Called every loop.
static void update_hall() {
    // Snapshot volatile state atomically before processing.
    uint32_t lastPulseUs = _hallLastPulseUs;
    uint32_t periodUs    = _hallPeriodUs;
    uint32_t pulseCount  = _hallPulseCount;

    // Odometry: count new pulses since last call; skip accumulation during cogging.
    uint32_t newPulses    = pulseCount - _hallLastCountTracked;
    _hallLastCountTracked = pulseCount;

    // Pulse arrival rate — updated before the early returns below so the rate
    // decays to 0 at standstill instead of freezing at its last value.
    _hallHz.update(newPulses);
    _rpmData.hallHz = _hallHz.hz;
    if (!_rpmData.cogging)
        _rpmData.odometryCm += (float)newPulses * RPM_HALL_CM_PER_PULSE;
    _rpmData.timestamp = millis();

    bool stale = (micros() - lastPulseUs) > (RPM_STALE_MS * 1000UL);

    if (_rpmData.cogging || stale || periodUs == 0) {
        _hallEmaSpeed    = 0.0f;
        _hallSpikeStreak = 0;
        _rpmData.hallRawMph   = 0.0f;
        _rpmData.hallSpeedMph = 0.0f;
        // Hall silence above the encoder ramp is a measurement (≈0 mph) — without it
        // the fused KF deadlocks after a hard stop: encoder gated out, hall silent.
        // Below the ramp the encoder handles decay; loop-rate zeros would out-vote it.
        if (fabsf(_fused2Kf.angle) > rtConfig.fusedRampStartMph) {
            _fused2Kf.correct(0.0f, rtConfig.fusedHallR);
            _rpmData.fusedSpeedMph = _fused2Kf.angle;
        }
        return;
    }

    // Only process speed update when a new pulse has arrived — applying EMA every loop
    // against an unchanged periodUs would converge instantly and provide no smoothing.
    if (newPulses == 0) return;

    float periodS  = periodUs / 1000000.0f;
    float motorRpm = (60.0f / periodS) / RPM_PULSES_PER_REV;
    float rawSpeed = motorRpm * RPM_HALL_SPEED_FACTOR;

    // Reject noise pulses that produce an implausibly large speed jump.
    // Force-accept after RPM_SPIKE_MAX_STREAK consecutive rejections so genuine acceleration isn't blocked.
    bool spike = (_hallEmaSpeed > 0.0f &&
                  rawSpeed > _rpmData.hallSpeedMph * RPM_SPIKE_REJECT_FACTOR &&
                  _hallSpikeStreak < RPM_SPIKE_MAX_STREAK);
    if (spike) {
        ESP_LOGW(TAG, "⚠️ hall spike rejected: raw=%.2f mph  filtered=%.2f mph  streak=%d",
                 rawSpeed, _rpmData.hallSpeedMph, _hallSpikeStreak + 1);
        _hallSpikeStreak++;
        return;
    }

    _hallEmaSpeed    += RPM_EMA_ALPHA * (rawSpeed - _hallEmaSpeed);
    _hallSpikeStreak  = 0;
    _rpmData.hallRawMph   = rawSpeed;
    _rpmData.hallSpeedMph = _hallEmaSpeed;

    // Fused-speed hall correction per accepted pulse — raw speed, not the EMA:
    // the KF smooths on its own, and double-filtering would lag the high-speed input.
    _fused2Kf.correct(rawSpeed, rtConfig.fusedHallR);
    _rpmData.fusedSpeedMph = _fused2Kf.angle;
}

// =============================================================================
// Public API
// =============================================================================

// Initializes the hall-effect interrupt and probes the AS5600 encoder over I2C.
void rpm_init() {
    // Hall-effect: configure pin and attach falling-edge interrupt.
    pinMode(PIN_RPM, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RPM), on_hall_pulse, FALLING);
    Serial.printf("[%s] ✅ hall-effect sensor ready on pin %d\n", TAG, PIN_RPM);

    // Encoder: Wire already open from setup(); just probe the device.
    Wire.beginTransmission(AS5600_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.printf("[%s] ❌ AS5600 not found at 0x%02X — check wiring\n", TAG, AS5600_ADDR);
    } else {
        Serial.printf("[%s] ✅ AS5600 encoder ready at 0x%02X (cogging detection)\n", TAG, AS5600_ADDR);
        check_magnet_status();
    }
}

// Updates hall-effect speed/odometry and encoder cogging detection; returns true when
// a new valid encoder angle was read this call.
bool rpm_update(float fwdAccelMps2) {
    // 2-state predict before corrections: integrate bias-corrected forward accel
    // (m/s² → mph/s) at the 50 Hz accel-report cadence.
    float dt2;
    if (_fused2PredictGate.tick(dt2) && isfinite(fwdAccelMps2)) {
        _fused2Kf.predict(fwdAccelMps2 * 2.23694f, dt2, rtConfig.fused2QSpeed, rtConfig.fused2QBias);
        _rpmData.fusedSpeedMph = _fused2Kf.angle;
    }

    // True when a fresh AS5600 angle was read this call (poll gate fired and the I2C
    // read succeeded) — main.cpp counts these to report the encoder sample rate.
    bool encSampled = update_encoder();
    update_hall();

    static uint32_t lastLogMs = 0;
    uint32_t now = millis();
    if (now - lastLogMs >= 5) {
        lastLogMs = now;
        // ESP_LOGI(TAG, "speed=%.2fmph  encVel=%.3fmph  cogging=%d  sc=%d",
        //          _rpmData.hallSpeedMph, _encEmaVelocityMph, (int)_rpmData.cogging, _rpmData.signChanges);
    }

    return encSampled;
}

// Returns the latest RPM reading.
const RPMData& rpm_get() { return _rpmData; }
