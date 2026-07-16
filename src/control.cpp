// Throttle and steering control layer.
// Owns the control mode (SETPOINT / DIRECT / STOPPED — set by the dashboard, never by
// serial frames) and computes normalized [-1, 1] throttle and steering outputs from
// Pi commands and PID controllers. Has no knowledge of PWM, deadband, smoothing, or
// servo hardware — all of that lives in actuators.cpp.
#include "control.h"
#include "actuators.h"
#include "config.h"
#include "runtime_config.h"
#include "utils.h"
#include "rpm.h"
#include "imu.h"
#include "serial_hal.h"
#include <Arduino.h>
#include "esp_log.h"

static const char *TAG = "control";

static ControlOutput  _controlOutput = {0.0f, 0.0f};
// Output limits feed the PIDs' anti-windup: throttle is one-sided (no reverse in
// SETPOINT), steering keeps the symmetric [-1, 1] default.
static PidController  _throttlePid  = { THROTTLE_PID_KP,  THROTTLE_PID_KI,  THROTTLE_PID_KD, 0.0f, 1.0f };
static PidController  _steeringPid  = { STEERING_PID_KP,  STEERING_PID_KI,  STEERING_PID_KD  };
static ControlMode    _mode         = ControlMode::STOPPED;  // main.cpp sets DEFAULT_CONTROL_MODE after init
static ControlMode    _prevMode     = ControlMode::STOPPED;
static RateGate       _gate{ CONTROL_UPDATE_INTERVAL_MS };
// SETPOINT held heading (deg): seeded from boot yaw on the first update with a valid IMU
// reading, then overwritten by each valid command frame. NAN until first seeded.
static float          _setpointHeadingDeg = NAN;

// Human-readable mode name for logs and telemetry.
const char* control_mode_str(ControlMode m) {
    switch (m) {
        case ControlMode::SETPOINT:  return "SETPOINT";
        case ControlMode::DIRECT:  return "DIRECT";
        case ControlMode::STOPPED: return "STOPPED";
    }
    return "UNKNOWN";
}

// Sets the control mode. Dashboard /mode is the sole caller — the mode authority.
void control_set_mode(ControlMode m) { _mode = m; }

// Returns the current control mode.
ControlMode control_mode() { return _mode; }

// Returns the latest normalized throttle and steering output.
const ControlOutput& control_get() { return _controlOutput; }

// Returns the SETPOINT held target heading (deg): boot-seeded, overwritten by each valid
// command frame; NAN until first seeded. Read by the OLED/dashboard heading arrow.
float control_setpoint_heading_deg() { return _setpointHeadingDeg; }

// Initializes the actuator layer and LED indicator pin, and seeds the SETPOINT held heading.
void control_init() {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    actuators_init();
    // Seed the SETPOINT held heading from boot yaw. imu_init() blocks until the first
    // rotation vector arrives, so yaw is valid here.
    const ImuData& imu = imu_get();
    // timestamp != 0 is the real "IMU delivered data" signal — a dead IMU leaves yaw
    // zero-initialized (not NaN), so the isnan check alone would seed heading 0.
    if (imu.timestamp != 0 && !isnan(imu.yaw)) {
        _setpointHeadingDeg = imu.yaw;
        Serial.printf("[%s] ✅ control ready — SETPOINT heading seeded from boot yaw=%.1f°\n", TAG, imu.yaw);
    } else {
        Serial.printf("[%s] ✅ control ready — ⚠️ no IMU yaw at init, SETPOINT steering inactive until first command\n", TAG);
    }
}

// Runs PID controllers and mode logic, then sends normalized outputs to actuators.
bool control_update() {
    float dt;
    bool pidTick = _gate.tick(dt);

    // Apply runtime-tunable gains from dashboard
    _throttlePid.kp = rtConfig.kp;
    _throttlePid.ki = rtConfig.ki;
    _throttlePid.kd = rtConfig.kd;
    _steeringPid.kp = rtConfig.steeringKp;
    _steeringPid.ki = rtConfig.steeringKi;

    // Step 1 & 2: determine intent — set targetSpeed and steeringSetpoint
    float targetSpeed      = NAN;   // NAN → bypass throttle PID
    float steeringSetpoint = NAN;   // NAN → bypass steering PID
    float steeringMeasure  = 0.0f;
    bool  directMode       = false; // true → skip PID, _controlOutput already set

    // Reset steering PID and log on every mode transition.
    if (_mode != _prevMode) {
        ESP_LOGW(TAG, "🔀 control mode: %s → %s", control_mode_str(_prevMode), control_mode_str(_mode));
        _steeringPid.reset();
    }
    _prevMode = _mode;

    //
    switch (_mode) {
        case ControlMode::SETPOINT: {
            // Pi-commanded setpoints. On cmd timeout: throttle → neutral (NaN target
            // resets the PID), but steering keeps holding the last heading — centering
            // mid-turn on a comms blip would make the car plow straight.
            const CommandData& cmd = serial_hal_get();
            if (cmd.lastCmdMs != 0 && millis() - cmd.lastCmdMs <= CMD_TIMEOUT_MS) {
                targetSpeed       = cmd.targetSpeedMph;
                _setpointHeadingDeg = cmd.targetHeadingDeg;
            }
            if (!isnan(_setpointHeadingDeg)) {
                // Setpoint 0, measure = heading error wrapped to [-180, 180] (short way
                // across the 0/360 seam).
                float err = imu_get().yaw - _setpointHeadingDeg;
                while (err >  180.0f) err -= 360.0f;
                while (err < -180.0f) err += 360.0f;
                steeringSetpoint = 0.0f;
                steeringMeasure  = err;
            }
            break;
        }
        case ControlMode::DIRECT: {
            // Pi-commanded raw efforts — PIDs bypassed; actuators still apply deadband/
            // scale/smoothing. On cmd timeout: throttle cut, steering deliberately not
            // written so it holds position (same rationale as SETPOINT's heading hold).
            const CommandData& cmd = serial_hal_get();
            directMode = true;
            // Edge detector so each failsafe event logs once, with the measured frame
            // gap — distinguishes delivery stalls (gap barely over budget) from a
            // stopped sender (gap keeps climbing).
            static bool _directCmdFresh = false;
            if (cmd.lastDirectMs != 0 && millis() - cmd.lastDirectMs <= CMD_TIMEOUT_MS) {
                _controlOutput.throttle = cmd.directThrottle;
                _controlOutput.steering = cmd.directSteering;
                _directCmdFresh = true;
            } else {
                if (_directCmdFresh)
                    ESP_LOGW(TAG, "⏱️ DIRECT cmd timeout — throttle cut (last frame %lums ago)",
                             (unsigned long)(millis() - cmd.lastDirectMs));
                _directCmdFresh = false;
                _controlOutput.throttle = 0.0f;
            }
            _controlOutput.targetSpeedMph = 0.0f;  // no speed setpoint in DIRECT
            break;
        }
        case ControlMode::STOPPED:
            // Kill switch: both setpoints stay NaN → PIDs reset, both axes neutral below.
            break;
    }

    // Step 3 & 4: run PIDs or zero outputs (skipped in direct-output modes)
    if (!directMode) {
        if (!isnan(steeringSetpoint)) {
            if (pidTick)
                _controlOutput.steering = _steeringPid.update(steeringSetpoint, steeringMeasure, dt);
            // else hold last steering until next pidTick
        } else {
            _steeringPid.reset();
            _controlOutput.steering = 0.0f;
        }
        _controlOutput.targetSpeedMph = isnan(targetSpeed) ? 0.0f : targetSpeed;
        if (!isnan(targetSpeed)) {
            if (pidTick) {
                // Normalize to [0,1] so PID gains are independent of the speed scale.
                // Measure is the 2-state fused speed (accel-predicted, encoder/hall
                // corrected). Output arrives already clamped by the PID's outMin/outMax.
                float norm = rtConfig.maxSpeedMph > 0.0f ? rtConfig.maxSpeedMph : 1.0f;
                _controlOutput.throttle = _throttlePid.update(targetSpeed / norm, rpm_get().fusedSpeedMph / norm, dt);
            }
            // else hold last throttle until next pidTick
        } else {
            _throttlePid.reset();
            _controlOutput.throttle = 0.0f;
        }
    }

    // LED on whenever the car is trying to drive: nonzero set speed, or nonzero
    // direct throttle in DIRECT mode (which bypasses the target-speed path).
    bool driving = directMode ? (_controlOutput.throttle != 0.0f)
                              : (_controlOutput.targetSpeedMph > 0.0f);
    digitalWrite(PIN_LED, driving ? HIGH : LOW);

    actuators_set(_controlOutput.throttle, _controlOutput.steering);
    return pidTick;
}
