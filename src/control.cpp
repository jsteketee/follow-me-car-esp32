// ESC and steering servo control.
// Consume navigation mode
// Calculate error from targets
// Run PIDs
// Control Vehicle
#include "control.h"
#include "nav.h"
#include "fusion.h"
#include "config.h"
#include "runtime_config.h"
#include "utils.h"
#include "rpm.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include "esp_log.h"

static const char *TAG = "control";

#define PWM_NEUTRAL_US 1500
#define PWM_MIN_US     1000
#define PWM_MAX_US     2000



static Servo          escServo;
static Servo          steerServo;
static ControlOutput  _controlOutput = {0.0f, 0.0f};
static PidController  _throttlePid  = { THROTTLE_PID_KP,  THROTTLE_PID_KI,  THROTTLE_PID_KD  };
static PidController  _steeringPid  = { STEERING_PID_KP,  STEERING_PID_KI,  STEERING_PID_KD  };
static NavMode        _prevNavMode    = NavMode::STOPPED;
static RateGate       _gate{ CONTROL_UPDATE_INTERVAL_MS };


const ControlOutput& control_get() { return _controlOutput; }

void control_init() {
    esp_log_level_set("ESP32Servo.cpp", ESP_LOG_ERROR);
    esp_log_level_set("ESP32PWM.cpp",   ESP_LOG_ERROR);

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    escServo.attach(PIN_ESC,   PWM_MIN_US, PWM_MAX_US);
    steerServo.attach(PIN_SERVO, PWM_MIN_US, PWM_MAX_US);
    escServo.writeMicroseconds(PWM_NEUTRAL_US);
    steerServo.writeMicroseconds(PWM_NEUTRAL_US);

    ESP_LOGI(TAG, "[control] ✅ ESC (pin %d) + Servo (pin %d) ready — neutral %d µs",
             PIN_ESC, PIN_SERVO, PWM_NEUTRAL_US);
}

static int float_to_pwm(float val) {
    return (int)(PWM_NEUTRAL_US + constrain(val, -1.0f, 1.0f) * 500.0f);
}

static void control_apply() {
    float steering = constrain(_controlOutput.steering, -rtConfig.steeringMax, rtConfig.steeringMax);
    escServo.writeMicroseconds(constrain(float_to_pwm(_controlOutput.throttle), PWM_MIN_US, PWM_MAX_US));
    steerServo.writeMicroseconds(constrain(float_to_pwm(steering), PWM_MIN_US, PWM_MAX_US));
}

void control_update() {
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
    _lastValidAngle = pose.angle;
    // }

    // Step 1 & 2: determine intent — set targetSpeed and steeringSetpoint
    float targetSpeed      = NAN;   // NAN → bypass throttle PID
    float steeringSetpoint = NAN;   // NAN → bypass steering PID
    float steeringMeasure  = 0.0f;

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
            if (nav.sensorsValid && pose.distanceCm > rtConfig.followDistanceCm)
                targetSpeed = rtConfig.targetSpeedMph;
            break;
        case NavMode::TEST:
            steeringSetpoint = 0.0f;
            steeringMeasure  = _lastValidAngle;
            targetSpeed = rtConfig.targetSpeedMph;
            break;
        case NavMode::STOPPED:
            break;
    }

    // Step 3 & 4: run PIDs or zero outputs
    if (!isnan(steeringSetpoint)) {
        if (pidTick)
            _controlOutput.steering = _steeringPid.update(steeringSetpoint, steeringMeasure, dt);
        // else hold last steering until next pidTick
    } else {
        _steeringPid.reset();
        _controlOutput.steering = 0.0f;
    }
    if (!isnan(targetSpeed)) {
        if (pidTick) {
            float pidOut = _throttlePid.update(targetSpeed, rpm_get().speedMph, dt);
            float ffOut  = targetSpeed * rtConfig.throttleFfK;
            _controlOutput.throttle = rtConfig.throttleDeadband + constrain(pidOut + ffOut, 0.0f, 1.0f) * (rtConfig.throttleScale - rtConfig.throttleDeadband);
        }
        // else hold last throttle until next pidTick
    } else {
        _throttlePid.reset();
        _controlOutput.throttle = 0.0f;
    }

    static float _smoothThrottle = 0.0f;
    if (nav.sensorsValid) {
        _smoothThrottle += rtConfig.smoothAlpha * (_controlOutput.throttle - _smoothThrottle);
    } else {
        _smoothThrottle = 0.0f;
    }
    _controlOutput.throttle = _smoothThrottle;

    control_apply();
}

