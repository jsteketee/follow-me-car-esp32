// ESC and steering servo control.
// Consume navigation mode
// Calculate error from targets
// Run PIDs
// Control Vehicle
#include "control.h"
#include "nav.h"
#include "imu.h"
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
static NavMode       _prevState    = NavMode::STOPPED;

// Normalise a heading difference to [-180, +180] to handle the 360→0 wraparound.
static float headingError(float target, float current) {
    float e = target - current;
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

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

void control_apply(const ControlOutput &out) {
    _controlOutput = out;
    float steering = constrain(out.steering, -STEERING_MAX, STEERING_MAX);
    escServo.writeMicroseconds(constrain(float_to_pwm(out.throttle), PWM_MIN_US, PWM_MAX_US));
    steerServo.writeMicroseconds(constrain(float_to_pwm(steering),   PWM_MIN_US, PWM_MAX_US));
}

void control_update() {
    const NavData& nav = nav_get();
    const ImuData& imu = imu_get();
    static RateGate gate{ CONTROL_UPDATE_INTERVAL_MS };
    float dt;
    bool pidTick = gate.tick(dt);

    digitalWrite(PIN_LED, nav.mode == NavMode::FOLLOW_ME ? HIGH : LOW);

    // Step 1 & 2: determine intent — set targetSpeed/directThrottle and steeringSetpoint/directSteering
    float targetSpeed      = NAN;   // NAN → bypass throttle PID
    float directThrottle   = 0.0f;
    float steeringSetpoint = NAN;   // NAN → bypass steering PID
    float steeringMeasure  = 0.0f;
    float directSteering   = 0.0f;

    // Reset steering PID on every mode transition.
    if (nav.mode != _prevState) {
        _steeringPid.reset();
    }
    _prevState = nav.mode;

    float holdErr = headingError(nav.headingHold, imu.yaw);

    switch (nav.mode) {
        case NavMode::FOLLOW_ME:
            //Todo Need to integrate heading change and use that to adjust steering between waypoint measurements.
            steeringSetpoint = 0.0f;
            steeringMeasure  = nav.relativeAngle;
            if (nav.distanceCm > rtConfig.followDistanceCm)
                targetSpeed = rtConfig.targetSpeedMph;
            break;
        case NavMode::TEST:
            targetSpeed      = rtConfig.targetSpeedMph;
            steeringSetpoint = imu.yaw + holdErr;
            steeringMeasure  = imu.yaw;
            break;
        case NavMode::STALE:
            steeringSetpoint = imu.yaw + holdErr;
            steeringMeasure  = imu.yaw;
            break;
        case NavMode::STOPPED:
            break;  // directThrottle = 0, directSteering = 0
    }

    // Step 3 & 4: run PIDs or apply direct values
    ControlOutput out = _controlOutput;
    if (!isnan(steeringSetpoint)) {
        if (pidTick) {
            out.steering = _steeringPid.update(steeringSetpoint, steeringMeasure, dt);
        }
        // else hold last steering until next pidTick
    } else {
        _steeringPid.reset();
        out.steering = directSteering;
    }
    if (!isnan(targetSpeed)) {
        _throttlePid.kp = rtConfig.kp;
        _throttlePid.ki = rtConfig.ki;
        _throttlePid.kd = rtConfig.kd;
        if (pidTick) {
            float pidOut = _throttlePid.update(targetSpeed, rpm_get().speedMph, dt);
            float ffOut  = targetSpeed * rtConfig.throttleFfK;
            out.throttle = rtConfig.throttleDeadband + constrain(pidOut + ffOut, 0.0f, 1.0f) * (rtConfig.throttleScale - rtConfig.throttleDeadband);
        }
        // else hold last throttle until next pidTick
    } else {
        _throttlePid.reset();
        out.throttle = directThrottle;
    }

    static float _smoothThrottle = 0.0f;
    _smoothThrottle += rtConfig.smoothAlpha * (out.throttle - _smoothThrottle);
    out.throttle = _smoothThrottle;

    control_apply(out);
}

