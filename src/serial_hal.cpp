// serial_hal — 50 Hz USB-CDC JSON telemetry stream for the Pi ROS2 bridge, plus the
// non-blocking RX path for command frames (protocol: follow-me-car-ros2/PROJECT_PLAN.md).
// Interleaves with ESP_LOG output; the Pi ignores any line not starting with '{'.
#include "serial_hal.h"
#include "uwb.h"
#include "imu.h"
#include "rpm.h"
#include "pan.h"
#include "control.h"
#include "utils.h"
#include "runtime_config.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

static RateGate _gate{20};  // 50 Hz

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

    if (!process_setpoint_frame(line)) process_direct_frame(line);
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

// Drains inbound command bytes every call, then emits one JSON telemetry frame if the 20 ms gate has elapsed.
void serial_hal_update() {
    serial_rx_drain();

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

    Serial.printf(
        "{\"ts\":%lu"
        ",\"uwb_dist\":%.1f,\"uwb_bearing\":%.2f,\"uwb_age\":%ld"
        ",\"yaw\":%.2f,\"yaw_rate\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"lax\":%.3f"
        ",\"speed\":%.3f,\"odo\":%.1f,\"cogging\":%d"
        ",\"mode\":\"%s\""
        ",\"cmd_speed\":%.2f,\"cmd_heading\":%.1f,\"cmd_pan\":%.1f,\"cmd_age\":%ld,\"cmd_rejects\":%lu"
        ",\"throttle\":%.3f,\"steering\":%.3f"
        ",\"pan_angle\":%.2f"
        "}\n",
        millis(),
        uwb.distCm, uwb.angleDeg, uwbAge,
        imu.yaw, imu.yawRate, imu.pitch, imu.roll, imu.lax,
        rpm.fusedSpeedMph, rpm.odometryCm, (int)rpm.cogging,
        control_mode_str(control_mode()),
        _cmdData.targetSpeedMph, _cmdData.targetHeadingDeg, _cmdData.targetPanDeg, cmdAge, (unsigned long)_cmdRejects,
        ctrl.throttle, ctrl.steering,
        pan_get_angle());
}
