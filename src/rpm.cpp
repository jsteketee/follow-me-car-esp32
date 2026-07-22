// RPM driver: hall-effect interrupt for speed; AS5600 encoder for odometry and cogging detection.
// Hall-effect is the source of truth for speed. The AS5600 encoder is the source of truth for
// odometry (signed, high-res) and detects motor cogging — oscillation at a reluctance point.
#include "rpm.h"
#include "config.h"
#include "runtime_config.h"
#include "utils.h"
#include "log_event.h"
#include "esp_log.h"
#include <Arduino.h>
#include <Wire.h>

static const char* TAG = "rpm";

// =============================================================================
// Hall-effect sensor (speed + odometry)
// =============================================================================
static volatile uint32_t _hallLastPulseUs = 0;   // last ACCEPTED pulse — speed period is measured from this
static volatile uint32_t _hallLastEdgeUs  = 0;   // last edge of ANY kind — re-arms the debounce quiet-gap timer
static volatile uint32_t _hallDebounceUs  = RPM_DEBOUNCE_MAX_US; // speed-adaptive quiet-gap window (µs), sized by the main loop
static volatile uint32_t _hallPeriodUs    = 0;
static volatile uint32_t _hallPulseCount  = 0;
static volatile bool     _hallPulseValid  = false;
static uint32_t          _hallLastCountTracked = 0; // pulse count snapshot for odometry
static HzTracker         _hallHz;                   // pulse arrival rate — 0 at standstill, scales with wheel speed
static uint32_t          _hallAccelLastUs  = 0;     // time + speed of the last accel-accepted reading (plausibility gate)
static float             _hallAccelLastMph = 0.0f;

// ISR — debounces, records inter-pulse period, and increments pulse counter.
static void IRAM_ATTR on_hall_pulse() {
    uint32_t now       = micros();
    uint32_t sinceEdge = now - _hallLastEdgeUs;
    _hallLastEdgeUs    = now;  // every edge re-arms the quiet-gap timer, accepted or not
    // A real magnet pass is an isolated edge preceded by a long quiet gap. Reject any edge that
    // follows another within the debounce window — measured from the last EDGE, not the last
    // accepted pulse, so sustained chatter yields zero pulses (a last-accepted debounce would
    // instead admit one edge per window: a steady phantom pulse train). The window is speed-
    // adaptive (_hallDebounceUs, sized by the main loop from fused speed): wide when slow so a
    // fast repeat is confidently a bounce, tight when fast so the next real pulse is never dropped.
    if (_hallPulseValid && sinceEdge < _hallDebounceUs) return;
    if (_hallPulseValid) _hallPeriodUs = now - _hallLastPulseUs;
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
static AngleKalman  _fused2KfNoImu;         // same filter, same encoder+hall corrections, but predicted with 0 accel (no IMU) — bench diagnostic
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

// Reads the STATUS MD bit; returns 1 if a magnet is detected, 0 if not, -1 on I2C error.
static int read_magnet_detected() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_STATUS);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(AS5600_ADDR, (uint8_t)1) != 1) return -1;
    return (Wire.read() & (1 << 5)) ? 1 : 0;
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
        _rpmData.encoderHealthy = false; // I2C unreachable — phantom-odom gate can't corroborate
        ESP_LOGW(TAG, "⚠️ AS5600 read failed");
        return false;
    }
    _rpmData.encAngle = angle;  // raw angle for the dashboard angle graph

    // Refresh encoder health at ~4 Hz: I2C just succeeded (angle read above), so health tracks
    // magnet presence. A lost magnet reads a stuck angle that looks "still" and would let the
    // phantom-odom gate veto real travel — dropping health here deactivates that gate instead.
    static uint32_t _lastHealthMs = 0;
    uint32_t nowHealth = millis();
    if (nowHealth - _lastHealthMs >= 250) {
        _lastHealthMs = nowHealth;
        int md = read_magnet_detected();
        if (md >= 0) _rpmData.encoderHealthy = (md == 1);
    }

    if (_encLastAngle < 0) {
        _encLastAngle = angle;
        return true;
    }

    // Resolve the 0/4095 wraparound. Below RPM_ALIAS_FWD_MIN_MPH direction is genuinely
    // ambiguous (cogging / real reverse), so take the blind shortest path. Above it we're
    // confidently rolling forward (a NEGATIVE count step here — see the -delta in the velocity
    // calc below); snap the raw delta to the whole-rev offset nearest the step our last fused
    // speed predicts, which de-aliases a >½-rev forward wrap (encoder aliases above ~7 mph)
    // without inventing motion from a small noise blip.
    int delta = angle - _encLastAngle;
    if (_rpmData.fusedSpeedMph > RPM_ALIAS_FWD_MIN_MPH) {
        float expected = -_rpmData.fusedSpeedMph * dt / (RPM_CM_PER_COUNT * 0.0223694f);
        while (delta - expected >  2048) delta -= AS5600_COUNTS_PER_REV;
        while (delta - expected < -2048) delta += AS5600_COUNTS_PER_REV;
    } else {
        if (delta >  2048) delta -= AS5600_COUNTS_PER_REV;
        if (delta < -2048) delta += AS5600_COUNTS_PER_REV;
    }
    _encLastAngle = angle;

    // Cumulative signed count for distance calibration — sums the shortest-path delta, so it
    // stays honest through slow-push jitter. Valid only below the ~7 mph alias limit (>½ rev per
    // 4 ms poll aliases); the calibration push must stay well under that.
    _rpmData.encCounts += delta;

    // Odometry source of truth: signed distance (forward = -delta, matching the velocity calc),
    // so reverse subtracts and small moves the coarse hall pulse count can't resolve still register.
    _rpmData.odometryCm += (float)(-delta) * RPM_CM_PER_COUNT;

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
    float fused2EncRNoImu = fused_enc_r(_fused2KfNoImu.angle);  // no-IMU twin, gated on its own estimate
    if (!isnan(fused2EncRNoImu)) {
        _fused2KfNoImu.correct(rawVelMph, fused2EncRNoImu);
        _rpmData.fusedNoImuMph = _fused2KfNoImu.angle;
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

// Reads latest hall-effect ISR state, updates speed, and accumulates odometry. Called every loop.
static void update_hall() {
    // Snapshot volatile state atomically before processing.
    uint32_t lastPulseUs = _hallLastPulseUs;
    uint32_t periodUs    = _hallPeriodUs;
    uint32_t pulseCount  = _hallPulseCount;

    uint32_t newPulses    = pulseCount - _hallLastCountTracked;
    _hallLastCountTracked = pulseCount;

    // Pulse arrival rate — updated before the early returns below so the rate
    // decays to 0 at standstill instead of freezing at its last value.
    _hallHz.update(newPulses);
    _rpmData.hallHz = _hallHz.hz;
    _rpmData.timestamp = millis();

    bool stale = (micros() - lastPulseUs) > (RPM_STALE_MS * 1000UL);

    if (_rpmData.cogging || stale || periodUs == 0) {
        _rpmData.hallRawMph   = 0.0f;
        _rpmData.hallSpeedMph = 0.0f;
        // Hall silence above the encoder ramp is a measurement (≈0 mph) — without it
        // the fused KF deadlocks after a hard stop: encoder gated out, hall silent.
        // Below the ramp the encoder handles decay; loop-rate zeros would out-vote it.
        if (fabsf(_fused2Kf.angle) > rtConfig.fusedRampStartMph) {
            _fused2Kf.correct(0.0f, rtConfig.fusedHallR);
            _rpmData.fusedSpeedMph = _fused2Kf.angle;
        }
        if (fabsf(_fused2KfNoImu.angle) > rtConfig.fusedRampStartMph) {  // no-IMU twin
            _fused2KfNoImu.correct(0.0f, rtConfig.fusedHallR);
            _rpmData.fusedNoImuMph = _fused2KfNoImu.angle;
        }
        return;
    }

    // Only process the speed update when a new pulse has arrived — periodUs is unchanged otherwise.
    if (newPulses == 0) return;

    float periodS  = periodUs / 1000000.0f;
    float motorRpm = (60.0f / periodS) / RPM_PULSES_PER_REV;
    float rawSpeed = motorRpm * RPM_HALL_SPEED_FACTOR;

    // Physical-plausibility gate: reject a reading whose implied acceleration exceeds the car's
    // max — a double-count/glitch gives a short period and thus an impossible |Δspeed/Δt|. Judged
    // against the last accepted reading, so it self-corrects after a rejected pulse. (A more
    // sophisticated version could scale the limit by the measured IMU forward accel.)
    float accelDt = (lastPulseUs - _hallAccelLastUs) / 1000000.0f;
    if (_hallAccelLastUs != 0 && accelDt > 0.0f &&
        fabsf(rawSpeed - _hallAccelLastMph) / accelDt > RPM_HALL_MAX_ACCEL_MPH_S) {
        static uint32_t _lastAccelRejMs = 0;
        if (millis() - _lastAccelRejMs >= 1000) {
            _lastAccelRejMs = millis();
            ESP_LOGW(TAG, "⚠️ hall reading rejected (impossible accel): raw=%.2f last=%.2f dt=%.3fs → %.0f mph/s",
                     rawSpeed, _hallAccelLastMph, accelDt, fabsf(rawSpeed - _hallAccelLastMph) / accelDt);
        }
        return;
    }
    _hallAccelLastMph = rawSpeed;
    _hallAccelLastUs  = lastPulseUs;
    _rpmData.hallPulses += newPulses;  // count only accel-accepted pulses (dashboard tick markers)

    // Speed channel: raw per-pulse speed, unsmoothed and never phantom-gated. Under-reporting
    // speed makes the throttle PID over-command (unintended acceleration), so speed fails high —
    // the ISR debounce already dropped physically-impossible edges.
    _rpmData.hallRawMph   = rawSpeed;
    _rpmData.hallSpeedMph = rawSpeed;

    // Fused-speed hall correction per debounced pulse; the KF does its own smoothing.
    _fused2Kf.correct(rawSpeed, rtConfig.fusedHallR);
    _rpmData.fusedSpeedMph = _fused2Kf.angle;
    _fused2KfNoImu.correct(rawSpeed, rtConfig.fusedHallR);  // no-IMU twin
    _rpmData.fusedNoImuMph = _fused2KfNoImu.angle;
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
        _rpmData.encoderHealthy = false;
        log_event(LOG_WARN, "encoder not found — odom unguarded");
    } else {
        Serial.printf("[%s] ✅ AS5600 encoder ready at 0x%02X (cogging detection)\n", TAG, AS5600_ADDR);
        check_magnet_status();
        _rpmData.encoderHealthy = (read_magnet_detected() == 1);
    }
}

// Updates hall-effect speed/odometry and encoder cogging detection; returns true when
// a new valid encoder angle was read this call.
bool rpm_update(float fwdAccelMps2) {
    // 2-state predict before corrections: integrate bias-corrected forward accel
    // (m/s² → mph/s) at the 50 Hz accel-report cadence.
    float dt2;
    if (_fused2PredictGate.tick(dt2)) {
        if (isfinite(fwdAccelMps2)) {
            _fused2Kf.predict(fwdAccelMps2 * 2.23694f, dt2, rtConfig.fused2QSpeed, rtConfig.fused2QBias);
            _rpmData.fusedSpeedMph = _fused2Kf.angle;
        }
        // No-IMU twin: same predict cadence but 0 accel (constant-velocity model).
        _fused2KfNoImu.predict(0.0f, dt2, rtConfig.fused2QSpeed, rtConfig.fused2QBias);
        _rpmData.fusedNoImuMph = _fused2KfNoImu.angle;
    }

    // True when a fresh AS5600 angle was read this call (poll gate fired and the I2C
    // read succeeded) — main.cpp counts these to report the encoder sample rate.
    bool encSampled = update_encoder();
    update_hall();

    // Size the ISR debounce window from the current fused speed: a fraction of the real pulse
    // period at that speed, clamped. Wide at rest (a fast repeat is bounce), tight at speed (never
    // drop the closely-spaced next real pulse). Published to the volatile the ISR reads.
    float v = fabsf(_fused2Kf.angle);
    uint32_t win = (uint32_t)rtConfig.debounceMaxUs;
    if (v > 0.01f) {
        float periodUs = 60.0e6f * RPM_HALL_SPEED_FACTOR / (v * RPM_PULSES_PER_REV);
        float w = rtConfig.debounceSpeedFactor * periodUs;
        win = (uint32_t)fminf(fmaxf(w, rtConfig.debounceMinUs), rtConfig.debounceMaxUs);
    }
    _hallDebounceUs = win;

    // Encoder-health transitions as events. The enc_fault telemetry flag carries the steady
    // state; this flags the moment it flips (e.g. a magnet lost mid-drive), once each way.
    // First call seeds from the boot state so a boot-time absence isn't double-reported
    // (rpm_init already emitted it).
    static bool _encHealthSeen = false;
    static bool _encHealthInit = false;
    if (!_encHealthInit) {
        _encHealthSeen = _rpmData.encoderHealthy;
        _encHealthInit = true;
    } else if (_rpmData.encoderHealthy != _encHealthSeen) {
        _encHealthSeen = _rpmData.encoderHealthy;
        if (_rpmData.encoderHealthy) log_event(LOG_WARN,  "encoder recovered");
        else                         log_event(LOG_ERROR, "encoder fault — odom unguarded");
    }

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
