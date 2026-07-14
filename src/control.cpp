// Throttle and steering control layer.
// Owns the control mode (REMOTE / DIRECT / STOPPED — set by the dashboard, never by
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
static PidController  _throttlePid  = { THROTTLE_PID_KP,  THROTTLE_PID_KI,  THROTTLE_PID_KD  };
static PidController  _steeringPid  = { STEERING_PID_KP,  STEERING_PID_KI,  STEERING_PID_KD  };
static ControlMode    _mode         = ControlMode::STOPPED;  // main.cpp sets DEFAULT_CONTROL_MODE after init
static ControlMode    _prevMode     = ControlMode::STOPPED;
static RateGate       _gate{ CONTROL_UPDATE_INTERVAL_MS };
// REMOTE held heading (deg): seeded from boot yaw on the first update with a valid IMU
// reading, then overwritten by each valid command frame. NAN until first seeded.
static float          _remoteHeadingDeg = NAN;

// Human-readable mode name for logs.
static const char* control_mode_str(ControlMode m) {
    switch (m) {
        case ControlMode::REMOTE:  return "REMOTE";
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

// Returns the REMOTE held target heading (deg): boot-seeded, overwritten by each valid
// command frame; NAN until first seeded. Read by the OLED/dashboard heading arrow.
float control_remote_heading_deg() { return _remoteHeadingDeg; }

// Initializes the actuator layer and LED indicator pin, and seeds the REMOTE held heading.
void control_init() {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    actuators_init();
    // Seed the REMOTE held heading from boot yaw. imu_init() blocks until the first
    // rotation vector arrives (same guarantee fusion_init() relies on), so yaw is valid here.
    const ImuData& imu = imu_get();
    if (!isnan(imu.yaw)) {
        _remoteHeadingDeg = imu.yaw;
        Serial.printf("[%s] ✅ control ready — REMOTE heading seeded from boot yaw=%.1f°\n", TAG, imu.yaw);
    } else {
        Serial.printf("[%s] ✅ control ready — ⚠️ no IMU yaw at init, REMOTE steering inactive until first command\n", TAG);
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
        case ControlMode::REMOTE: {
            // Pi-commanded setpoints from the serial RX path. The held heading is seeded
            // from boot yaw in control_init() and overwritten by each valid command frame.
            // Cmd-timeout failsafe: if no valid frame for CMD_TIMEOUT_MS (or none ever),
            // throttle goes neutral (targetSpeed stays NaN → throttle PID resets) but
            // steering STAYS ACTIVE, holding the last heading — centering the wheels
            // mid-turn on a comms blip would make the car plow straight. The car stays in
            // REMOTE and throttle auto-resumes on the next valid frame.
            const CommandData& cmd = serial_hal_get();
            if (cmd.lastCmdMs != 0 && millis() - cmd.lastCmdMs <= CMD_TIMEOUT_MS) {
                targetSpeed       = cmd.targetSpeedMph;
                _remoteHeadingDeg = cmd.targetHeadingDeg;
            }
            if (!isnan(_remoteHeadingDeg)) {
                // Setpoint 0, measure = heading error wrapped to [-180, 180] (short way
                // across the 0/360 seam), same while-loop pattern as fusion.cpp correct_bearing().
                float err = imu_get().yaw - _remoteHeadingDeg;
                while (err >  180.0f) err -= 360.0f;
                while (err < -180.0f) err += 360.0f;
                steeringSetpoint = 0.0f;
                steeringMeasure  = err;
            }
            break;
        }
        case ControlMode::DIRECT: {
            // Pi-commanded raw actuator efforts — PIDs bypassed; actuators still apply
            // deadband/scale/smoothing. Cmd-timeout failsafe: throttle cut, steering
            // deliberately NOT written so it holds the last commanded position (same
            // rationale as REMOTE's heading hold).
            const CommandData& cmd = serial_hal_get();
            directMode = true;
            if (cmd.lastDirectMs != 0 && millis() - cmd.lastDirectMs <= CMD_TIMEOUT_MS) {
                _controlOutput.throttle = cmd.directThrottle;
                _controlOutput.steering = cmd.directSteering;
            } else {
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
                // Measure is the hall-effect speed directly (was fusion's fusedSpeedMph,
                // which merely plumbed the same value through — re-pointed ahead of the
                // planned fusion strip, per PROJECT_PLAN Phase 2).
                float norm   = rtConfig.maxSpeedMph > 0.0f ? rtConfig.maxSpeedMph : 1.0f;
                float pidOut = _throttlePid.update(targetSpeed / norm, rpm_get().speedMph / norm, dt);
                _controlOutput.throttle = constrain(pidOut, 0.0f, 1.0f);
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
