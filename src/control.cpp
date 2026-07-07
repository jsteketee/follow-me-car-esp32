// Throttle and steering control layer.
// Computes normalized [-1, 1] throttle and steering outputs from navigation mode,
// sensor fusion data, and PID controllers. Has no knowledge of PWM, deadband,
// smoothing, or servo hardware — all of that lives in actuators.cpp.
#include "control.h"
#include "actuators.h"
#include "nav.h"
#include "fusion.h"
#include "config.h"
#include "runtime_config.h"
#include "utils.h"
#include "rpm.h"
#include <Arduino.h>
#include "esp_log.h"

static const char *TAG = "control";

static ControlOutput  _controlOutput = {0.0f, 0.0f};
static PidController  _throttlePid  = { THROTTLE_PID_KP,  THROTTLE_PID_KI,  THROTTLE_PID_KD  };
static PidController  _steeringPid  = { STEERING_PID_KP,  STEERING_PID_KI,  STEERING_PID_KD  };
static NavMode        _prevNavMode    = NavMode::STOPPED;
static RateGate       _gate{ CONTROL_UPDATE_INTERVAL_MS };

// Returns the latest normalized throttle and steering output.
const ControlOutput& control_get() { return _controlOutput; }

// Initializes the actuator layer and LED indicator pin.
void control_init() {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    actuators_init();
    ESP_LOGI(TAG, "✅ control ready");
}

// Runs PID controllers and mode logic, then sends normalized outputs to actuators.
bool control_update() {
    const NavData&   nav   = nav_get();
    const Pose& pose = fusion_get();
    float dt;
    bool pidTick = _gate.tick(dt);

    // Apply runtime-tunable gains from dashboard
    _throttlePid.kp = rtConfig.kp;
    _throttlePid.ki = rtConfig.ki;
    _throttlePid.kd = rtConfig.kd;
    _steeringPid.kp = rtConfig.steeringKp;
    _steeringPid.ki = rtConfig.steeringKi;

    digitalWrite(PIN_LED, nav.mode == NavMode::FOLLOW_ME ? HIGH : LOW);

    // Freeze angle at last valid reading so steering holds course during sensor dropout.
    static float _lastValidAngle = 0.0f;
    // if (nav.sensorsValid) {
    _lastValidAngle = pose.fusedAngle;
    // }

    // Step 1 & 2: determine intent — set targetSpeed and steeringSetpoint
    float targetSpeed      = NAN;   // NAN → bypass throttle PID
    float steeringSetpoint = NAN;   // NAN → bypass steering PID
    float steeringMeasure  = 0.0f;
    bool  directMode       = false; // true → skip PID, _controlOutput already set

    // Reset steering PID on every mode transition.
    if (nav.mode != _prevNavMode) {
        _steeringPid.reset();
    }
    _prevNavMode = nav.mode;

    //
    switch (nav.mode) {
        case NavMode::FOLLOW_ME:
            //Todo Need to integrate heading change and use to adjust steering between waypoint measurements.
            steeringSetpoint = 0.0f;
            steeringMeasure  = _lastValidAngle;
            if (nav.sensorsValid && pose.distanceCm > rtConfig.followDistanceCm) {
                float t = constrain((pose.distanceCm - rtConfig.followDistanceCm) /
                                    (rtConfig.maxDistanceCm - rtConfig.followDistanceCm), 0.0f, 1.0f);
                targetSpeed = rtConfig.minSpeedMph + t * (rtConfig.maxSpeedMph - rtConfig.minSpeedMph);
            }
            break;
        case NavMode::TEST:
            steeringSetpoint = 0.0f;
            steeringMeasure  = _lastValidAngle;
            targetSpeed = rtConfig.maxSpeedMph;
            break;
        case NavMode::THROTTLE_TEST:
            // Direct throttle command — bypasses PID; actuators still apply deadband/scale/smoothing.
            _controlOutput.throttle = rtConfig.testThrottle;
            _controlOutput.steering = 0.0f;
            directMode = true;
            break;
        case NavMode::STOPPED:
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
                float norm   = rtConfig.maxSpeedMph > 0.0f ? rtConfig.maxSpeedMph : 1.0f;
                float pidOut = _throttlePid.update(targetSpeed / norm, pose.fusedSpeedMph / norm, dt);
                _controlOutput.throttle = constrain(pidOut, 0.0f, 1.0f);
            }
            // else hold last throttle until next pidTick
        } else {
            _throttlePid.reset();
            _controlOutput.throttle = 0.0f;
        }
    }

    actuators_set(_controlOutput.throttle, _controlOutput.steering);
    return pidTick;
}
