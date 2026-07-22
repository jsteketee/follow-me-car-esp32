// Hardware actuation layer for ESC and steering servo.
// Owns all hardware-specific concerns: servo objects, PWM conversion, deadband/scale
// mapping, EMA smoothing, steering trim, and output clamping. Control.cpp calls
// actuators_set() with normalized [-1, 1] values and has no knowledge of the hardware.
#include "actuators.h"
#include "config.h"
#include "runtime_config.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include "esp_log.h"

static const char* TAG = "actuators";

static Servo escServo;
static Servo steerServo;
static float _smoothThrottle = 0.0f;

// Last PWM µs actually written to each servo, exposed for telemetry so the Pi/dashboard
// can see the post-deadband/scale/smoothing output — one stage past control's normalized effort.
static int _escPwmUs   = PWM_NEUTRAL_US;
static int _steerPwmUs = PWM_NEUTRAL_US;

// Maps a [-1, 1] normalized value to a PWM microsecond value centered on neutral.
static int float_to_pwm(float val) {
    return (int)(PWM_NEUTRAL_US + constrain(val, -1.0f, 1.0f) * 500.0f);
}

// Attaches ESC and steering servo and arms both to neutral PWM.
void actuators_init() {
    esp_log_level_set("ESP32Servo.cpp", ESP_LOG_ERROR);
    esp_log_level_set("ESP32PWM.cpp",   ESP_LOG_ERROR);
    escServo.attach(PIN_ESC,    PWM_MIN_US, PWM_MAX_US);
    steerServo.attach(PIN_SERVO, PWM_MIN_US, PWM_MAX_US);
    escServo.writeMicroseconds(PWM_NEUTRAL_US);
    steerServo.writeMicroseconds(PWM_NEUTRAL_US);
    Serial.printf("[%s] ✅ ESC (pin %d) + Servo (pin %d) ready — neutral %d µs\n",
                  TAG, PIN_ESC, PIN_SERVO, PWM_NEUTRAL_US);
}

// Accepts [-1, 1] throttle and steering, applies all hardware transforms, and writes to servos.
void actuators_set(float throttle, float steering) {
    // Smooth throttle with EMA in normalized space before hardware mapping.
    _smoothThrottle += rtConfig.smoothAlpha * (throttle - _smoothThrottle);

    // Apply deadband and scale: t=0 → neutral (0), t>0 → jump to deadband then scale up to throttleScale.
    // The jump ensures any non-zero throttle immediately overcomes the ESC's dead zone.
    // The brake side (t<0) keeps the engagement jump but ignores throttleScale: the
    // scale limits propulsion power, while braking should reach full strength — so
    // t=-1 maps to the full -1.0 output (1000µs).
    float t = _smoothThrottle;

    float scaledThrottle;
    if (t > 0.0f) {
        scaledThrottle = rtConfig.throttleDeadband + t * (rtConfig.throttleScale - rtConfig.throttleDeadband);
    } else if (t < 0.0f) {
        scaledThrottle = -(rtConfig.throttleDeadband + (-t) * (1.0f - rtConfig.throttleDeadband));
    } else {
        scaledThrottle = 0.0f;
    }

    // Apply steering trim and clamp to physical max deflection.
    float scaledSteering = constrain(steering * rtConfig.steeringMax + rtConfig.steeringTrim, -1.0f, 1.0f);

    // Convert and write PWM to both servos.
    _escPwmUs   = constrain(float_to_pwm(scaledThrottle), PWM_MIN_US, PWM_MAX_US);
    _steerPwmUs = constrain(float_to_pwm(scaledSteering), PWM_MIN_US, PWM_MAX_US);
    escServo.writeMicroseconds(_escPwmUs);
    steerServo.writeMicroseconds(_steerPwmUs);
}

// Last PWM µs written to the ESC (throttle) and steering servo.
int actuators_get_esc_pwm()   { return _escPwmUs; }
int actuators_get_steer_pwm() { return _steerPwmUs; }
