// serial_hal — 50 Hz USB-CDC JSON telemetry stream for the Pi ROS2 bridge, plus the
// non-blocking RX path for command frames (protocol: follow-me-car-ros2/PROJECT_PLAN.md).
// Interleaves with ESP_LOG output; the Pi ignores any line not starting with '{'.
#include "serial_hal.h"
#include "uwb.h"
#include "imu.h"
#include "rpm.h"
#include "pan.h"
#include "actuators.h"
#include "control.h"
#include "utils.h"
#include "runtime_config.h"
#include "log_event.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

static RateGate _gate{20};          // 50 Hz telemetry
static RateGate _healthGate{1000};  // 1 Hz sensor-health frames

// Loop/UWB/encoder update rates (Hz) and worst loop-gap (µs) published by main each loop,
// reused for the health frame; IMU and hall rates are read live from their own modules at emit
// time. maxLoopUs is a running max across health-frame intervals (reset on emit, not per loop).
static struct { float loopHz, uwbHz, encHz; uint32_t maxLoopUs; } _healthData = { 0.0f, 0.0f, 0.0f, 0 };

// Latest validated command frames; timestamps stay 0 until the first valid frame of
// each shape arrives.
static CommandData _cmdData = { 0.0f, 0.0f, 0, 0.0f, 0.0f, 0, 0.0f };

// Command values rejected by validation since boot — echoed in telemetry as
// cmd_rejects so the Pi can distinguish frames-arriving-but-invalid (counter ticking)
// from frames not arriving at all (cmd_age climbing, counter flat).
static uint32_t _cmdRejects = 0;

// RX line assembly buffer. Frames are newline-delimited; anything that doesn't parse
// and validate as a command frame is silently dropped (logs, garbage, torn lines).
static char   _rxBuf[128];
static size_t _rxLen      = 0;
static bool   _rxOverflow = false;  // discarding until the next '\n' after buffer overflow

// Returns the latest validated command frame from the Pi.
const CommandData& serial_hal_get() { return _cmdData; }

// Caches main's loop/UWB/encoder rate counters and folds the loop-gap into a running max for
// the next health frame. maxLoopUs arrives as main's window running-max; taking the max of what
// we see between emits captures the peak even across main's per-window reset.
void serial_hal_set_health_rates(float loopHz, float uwbHz, float encHz, uint32_t maxLoopUs) {
    _healthData.loopHz = loopHz;
    _healthData.uwbHz  = uwbHz;
    _healthData.encHz  = encHz;
    if (maxLoopUs > _healthData.maxLoopUs) _healthData.maxLoopUs = maxLoopUs;
}

// Emits one sensor-health frame: measured update rates in Hz (0 = silent/dead). Built as a
// single whole line and dropped wholesale if the TX buffer can't hold it, so it never blocks
// or tears a telemetry frame. Loop task only — shares the telemetry TX owner.
static void emit_health_frame() {
    const ImuData& imu = imu_get();
    const RPMData& rpm = rpm_get();
    char line[192];
    int n = snprintf(line, sizeof(line),
        "{\"type\":\"health\",\"max_loop_us\":%lu,\"sensors\":{"
        "\"imu\":%.1f,\"uwb\":%.1f,\"enc\":%.1f,\"hall\":%.1f,\"loop\":%.1f}}\n",
        (unsigned long)_healthData.maxLoopUs,
        (float)imu.update_hz, _healthData.uwbHz, _healthData.encHz, rpm.hallHz, _healthData.loopHz);
    if (n <= 0) return;
    size_t len = (n < (int)sizeof(line)) ? (size_t)n : sizeof(line) - 1;
    if (Serial.availableForWrite() < (int)len) return;  // full TX buffer: drop, never block (max keeps accumulating)
    Serial.write((const uint8_t*)line, len);
    _healthData.maxLoopUs = 0;  // reset only after a frame actually goes out, so a dropped frame doesn't lose the peak
}

// Dashboard bench-test injection: stores a synthetic DIRECT frame exactly as if the
// Pi had sent it — same slot, same ranges as process_direct_frame, same lastDirectMs
// stamp feeding the cmd-timeout failsafe. If the Pi is streaming DIRECT frames at the
// same time, the two sources interleave (newest write wins); that's a bench-only setup.
void serial_hal_inject_direct(float throttle, float steering, float panDeg) {
    if (!isfinite(throttle) || !isfinite(steering) || !isfinite(panDeg)) { _cmdRejects++; return; }
    _cmdData.directThrottle = constrain(throttle, -1.0f, 1.0f);  // negative = brake/reverse, matching the Pi path
    _cmdData.directSteering = constrain(steering, -1.0f, 1.0f);
    _cmdData.targetPanDeg   = constrain(panDeg, -90.0f, 90.0f);  // same bound as the Pi's target_pan validation
    _cmdData.lastDirectMs   = millis();
    control_set_mode(ControlMode::DIRECT);  // auto-switch on shape, same as a Pi DIRECT frame (ignored if STOPPED-latched)
}

// Dashboard bench-test injection for SETPOINT: stores a synthetic setpoint frame —
// speed clamped to the same [0, rtConfig.maxSpeedMph] range process_setpoint_frame
// enforces, heading normalized to [0, 360) — and stamps lastCmdMs so the SETPOINT
// cmd-timeout failsafe governs it exactly like Pi frames.
void serial_hal_inject_setpoint(float speedMph, float headingDeg) {
    if (!isfinite(speedMph) || !isfinite(headingDeg)) { _cmdRejects++; return; }
    while (headingDeg >= 360.0f) headingDeg -= 360.0f;
    while (headingDeg <    0.0f) headingDeg += 360.0f;
    _cmdData.targetSpeedMph   = constrain(speedMph, 0.0f, rtConfig.maxSpeedMph);
    _cmdData.targetHeadingDeg = headingDeg;
    _cmdData.lastCmdMs        = millis();
    control_set_mode(ControlMode::SETPOINT);  // auto-switch on shape, same as a Pi SETPOINT frame (ignored if STOPPED-latched)
}

// Extracts the float following "key": in a JSON line; false if the key is absent or the number is malformed.
static bool json_get_float(const char* line, const char* key, float* out) {
    const char* p = strstr(line, key);
    if (!p) return false;
    p += strlen(key);
    char* end = nullptr;
    float v = strtof(p, &end);
    if (end == p) return false;  // no digits after the key (also rejects "":," etc.)
    *out = v;
    return true;
}

// Logs command accept/reject at most once per second so 20 Hz streams don't flood the log.
static void log_cmd(const char* kind, const char* verdict, float a, float b) {
    static uint32_t _lastCmdLogMs = 0;
    if (millis() - _lastCmdLogMs < 1000) return;
    _lastCmdLogMs = millis();
    Serial.printf("[serial_hal] %s cmd %s  %.2f / %.2f\n", kind, verdict, a, b);
}

// Parses and validates one complete line as a setpoint frame (SETPOINT); updates _cmdData
// only if both values are present, finite, and in range. Rejects the whole frame
// otherwise. Returns true when the line carried setpoint keys (even if rejected).
static bool process_setpoint_frame(const char* line) {
    float speed, heading;
    if (!json_get_float(line, "\"target_speed\":",   &speed))   return false;
    if (!json_get_float(line, "\"target_heading\":", &heading)) return false;

    // Validation: a NaN reaching the actuator smoothing EMA poisons it permanently
    // (see actuators.cpp float_to_pwm path), so non-finite or out-of-range values
    // reject the entire frame — lastCmdMs is NOT stamped.
    if (!isfinite(speed) || !isfinite(heading))            { _cmdRejects++; log_cmd("setpoint", "REJECT (non-finite)", speed, heading); return true; }
    if (speed < 0.0f || speed > rtConfig.maxSpeedMph)      { _cmdRejects++; log_cmd("setpoint", "REJECT (speed range)", speed, heading); return true; }  // no reverse
    if (heading < -360.0f || heading > 720.0f)             { _cmdRejects++; log_cmd("setpoint", "REJECT (heading range)", speed, heading); return true; }

    // Normalize heading to [0, 360) — same convention as the telemetry yaw field.
    while (heading >= 360.0f) heading -= 360.0f;
    while (heading <    0.0f) heading += 360.0f;

    _cmdData.targetSpeedMph   = speed;
    _cmdData.targetHeadingDeg = heading;
    _cmdData.lastCmdMs        = millis();
    log_cmd("setpoint", "OK", speed, heading);
    return true;
}

// Parses and validates one complete line as a raw-actuator frame (DIRECT): normalized
// throttle [-1, 1] (negative = brake/reverse) + steering [-1, 1]. Same
// reject-whole-frame rules as setpoint frames. Returns true when the line carried
// actuator keys.
static bool process_direct_frame(const char* line) {
    float throttle, steering;
    if (!json_get_float(line, "\"throttle\":", &throttle)) return false;
    if (!json_get_float(line, "\"steering\":", &steering)) return false;

    if (!isfinite(throttle) || !isfinite(steering)) { _cmdRejects++; log_cmd("direct", "REJECT (non-finite)", throttle, steering); return true; }
    if (throttle < -1.0f || throttle > 1.0f)        { _cmdRejects++; log_cmd("direct", "REJECT (throttle range)", throttle, steering); return true; }
    if (steering < -1.0f || steering > 1.0f)        { _cmdRejects++; log_cmd("direct", "REJECT (steering range)", throttle, steering); return true; }

    _cmdData.directThrottle = throttle;
    _cmdData.directSteering = steering;
    _cmdData.lastDirectMs   = millis();
    log_cmd("direct", "OK", throttle, steering);
    return true;
}

// Routes one complete line through the frame parsers. Both frame shapes are always
// parsed and stored regardless of the current control mode (the dashboard is the mode
// authority; control decides what to act on). Lines matching neither shape are
// silently dropped (logs, garbage, torn lines).
static void process_command_line(const char* line) {
    // Optional pan command, honored in any frame shape: aim the UWB anchor (deg,
    // 0 = car nose, +right). An absent field keeps the previous target; an invalid
    // value is counted as a reject without affecting the rest of the frame. Routed
    // to the pan module by main's loop.
    float pan;
    if (json_get_float(line, "\"target_pan\":", &pan)) {
        if (isfinite(pan) && fabsf(pan) <= 90.0f) _cmdData.targetPanDeg = pan;
        else                                      _cmdRejects++;
    }

    // Auto-switch control mode to match an accepted command's shape: a valid SETPOINT frame
    // selects SETPOINT, a valid DIRECT frame selects DIRECT — detected by which command
    // timestamp the frame advanced. STOPPED is latching (control_set_mode ignores the request),
    // so a command can never auto-escape the safety stop. Acted on at the next control tick.
    uint32_t prevCmdMs    = _cmdData.lastCmdMs;
    uint32_t prevDirectMs = _cmdData.lastDirectMs;

    if (!process_setpoint_frame(line)) process_direct_frame(line);

    if      (_cmdData.lastCmdMs    != prevCmdMs)    control_set_mode(ControlMode::SETPOINT);
    else if (_cmdData.lastDirectMs != prevDirectMs) control_set_mode(ControlMode::DIRECT);
}

// Drains all pending RX bytes into the line buffer, non-blocking; processes a frame per '\n'.
static void serial_rx_drain() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            if (!_rxOverflow && _rxLen > 0) {
                _rxBuf[_rxLen] = '\0';
                process_command_line(_rxBuf);
            }
            _rxLen      = 0;
            _rxOverflow = false;      // newline always resynchronizes
        } else if (c == '{') {
            _rxBuf[0]   = c;          // resync: a '{' always starts a fresh frame
            _rxLen      = 1;
            _rxOverflow = false;
        } else if (_rxOverflow) {
            // discarding until the next '\n' (or '{' resync above)
        } else if (_rxLen < sizeof(_rxBuf) - 1) {
            _rxBuf[_rxLen++] = c;
        } else {
            _rxOverflow = true;       // line too long — drop it wholesale
            _rxLen      = 0;
        }
    }
}

// Serial already opened by main.cpp setup(); nothing to do here.
void serial_hal_init() {}

// Substitutes 0 for a non-finite telemetry value (NaN/Inf), so a bad sensor reading can never
// reach the wire — interface.md Wire Rule #1 (a non-numeric numeric field crashes the bridge).
static float finite_or_zero(float v, bool* scrubbed) {
    if (isfinite(v)) return v;
    *scrubbed = true;
    return 0.0f;
}

// Drains inbound command bytes every call, then emits one JSON telemetry frame if the 20 ms gate has elapsed.
void serial_hal_update() {
    serial_rx_drain();

    // Surface command-validation rejects as an event. The cmd_rejects counter carries the
    // running total in telemetry; this adds a human-visible heads-up, throttled to at most
    // once per 3s while the count moves so a 20 Hz stream of bad frames can't flood.
    static uint32_t _rejectsSeen = 0;
    if (_cmdRejects != _rejectsSeen) {
        _rejectsSeen = _cmdRejects;
        log_event_throttled("cmd_reject", LOG_WARN, 3000, "command rejected (%lu total)", (unsigned long)_cmdRejects);
    }

#ifndef SERIAL_HAL_TX_DISABLED
    // Sensor-health frames on their own 1 Hz gate, independent of the 50 Hz telemetry gate below.
    float hdt;
    if (_healthGate.tick(hdt)) emit_health_frame();
#endif

    float dt;
    if (!_gate.tick(dt)) return;

#ifdef SERIAL_HAL_TX_DISABLED
    return;  // quiet bench build (env:car-quiet): uplink silenced, RX command parsing stays live
#endif

    const UWBReading&    uwb  = uwb_get();
    const ImuData&       imu  = imu_get();
    const RPMData&       rpm  = rpm_get();
    const ControlOutput& ctrl = control_get();

    // Command echo: age of the last accepted command frame (-1 = none since boot),
    // so the Pi can verify what the car is acting on and see the failsafe explicitly.
    long cmdAge = _cmdData.lastCmdMs == 0 ? -1 : (long)(millis() - _cmdData.lastCmdMs);

    // Raw-UWB freshness: uwb_dist/uwb_bearing hold their last value through ranging
    // dropouts and outlier rejections, so the Pi needs the fix age (-1 = none since
    // boot) to tell fresh bearings from stale ones.
    long uwbAge = uwb.timestamp == 0 ? -1 : (long)(millis() - uwb.timestamp);

    // Every float field is passed through finite_or_zero: a NaN/Inf from a faulting sensor or a
    // diverged filter is replaced with 0 rather than printed as "nan"/"inf" (int fields and the
    // -1 sentinels can't go non-finite). scrubbed flags whether any substitution happened.
    // Built into a buffer then dropped wholesale if the USB-CDC TX buffer can't hold the whole
    // frame, so a Pi reading in bursts back-pressures into a dropped frame rather than a blocking
    // Serial write that stalls the loop task (the Pi keys off ts/cmd_age and tolerates gaps).
    bool scrubbed = false;
    char frame[640];
    int n = snprintf(frame, sizeof(frame),
        "{\"ts\":%lu"
        ",\"uwb_dist\":%.1f,\"uwb_bearing\":%.2f,\"uwb_age\":%ld"
        ",\"yaw\":%.2f,\"yaw_rate\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"lax\":%.3f"
        ",\"speed\":%.3f,\"odo\":%.1f,\"cogging\":%d,\"enc_fault\":%d"
        ",\"mode\":\"%s\""
        ",\"cmd_speed\":%.2f,\"cmd_heading\":%.1f,\"cmd_pan\":%.1f,\"cmd_age\":%ld,\"cmd_rejects\":%lu"
        ",\"throttle\":%.3f,\"steering\":%.3f"
        ",\"esc_pwm\":%d,\"steer_pwm\":%d,\"pan_pwm\":%d"
        ",\"pan_angle\":%.2f"
        "}\n",
        millis(),
        finite_or_zero(uwb.distCm, &scrubbed), finite_or_zero(uwb.angleDeg, &scrubbed), uwbAge,
        finite_or_zero(imu.yaw, &scrubbed), finite_or_zero(imu.yawRate, &scrubbed), finite_or_zero(imu.pitch, &scrubbed), finite_or_zero(imu.roll, &scrubbed), finite_or_zero(imu.lax, &scrubbed),
        finite_or_zero(rpm.fusedSpeedMph, &scrubbed), finite_or_zero(rpm.odometryCm, &scrubbed), (int)rpm.cogging, (int)!rpm.encoderHealthy,
        control_mode_str(control_mode()),
        finite_or_zero(_cmdData.targetSpeedMph, &scrubbed), finite_or_zero(_cmdData.targetHeadingDeg, &scrubbed), finite_or_zero(_cmdData.targetPanDeg, &scrubbed), cmdAge, (unsigned long)_cmdRejects,
        finite_or_zero(ctrl.throttle, &scrubbed), finite_or_zero(ctrl.steering, &scrubbed),
        actuators_get_esc_pwm(), actuators_get_steer_pwm(), pan_get_pwm(),
        finite_or_zero(pan_get_angle(), &scrubbed));
    if (n > 0) {
        size_t len = (n < (int)sizeof(frame)) ? (size_t)n : sizeof(frame) - 1;
        if (Serial.availableForWrite() >= (int)len)  // full TX buffer: drop this frame, never block
            Serial.write((const uint8_t*)frame, len);
    }

    // Substituting silently would hide a real sensor/filter fault behind plausible zeros, so warn
    // the Pi (throttled to 3s) that at least one field was scrubbed this frame.
    if (scrubbed)
        log_event_throttled("telem_nan", LOG_WARN, 3000, "non-finite telemetry field scrubbed to 0");
}
