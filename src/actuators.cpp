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
    ESP_LOGI(TAG, "✅ ESC (pin %d) + Servo (pin %d) ready — neutral %d µs",
             PIN_ESC, PIN_SERVO, PWM_NEUTRAL_US);
}

// Accepts [-1, 1] throttle and steering, applies all hardware transforms, and writes to servos.
void actuators_set(float throttle, float steering) {
    // Smooth throttle with EMA in normalized space before hardware mapping.
    _smoothThrottle += rtConfig.smoothAlpha * (throttle - _smoothThrottle);

    // Apply deadband and scale: t=0 → neutral (0), t>0 → jump to deadband then scale up to throttleScale.
    // The jump ensures any non-zero throttle immediately overcomes the ESC's dead zone.
    float t = _smoothThrottle;

    float scaledThrottle;
    if (t > 0.0f) {
        scaledThrottle = rtConfig.throttleDeadband + t * (rtConfig.throttleScale - rtConfig.throttleDeadband);
    } else if (t < 0.0f) {
        scaledThrottle = -(rtConfig.throttleDeadband + (-t) * (rtConfig.throttleScale - rtConfig.throttleDeadband));
    } else {
        scaledThrottle = 0.0f;
    }

    // Apply steering trim and clamp to physical max deflection.
    float scaledSteering = constrain(steering * rtConfig.steeringMax + rtConfig.steeringTrim, -1.0f, 1.0f);

    // Convert and write PWM to both servos.
    int escPwm = constrain(float_to_pwm(scaledThrottle), PWM_MIN_US, PWM_MAX_US);
    escServo.writeMicroseconds(escPwm);
    steerServo.writeMicroseconds(constrain(float_to_pwm(scaledSteering), PWM_MIN_US, PWM_MAX_US));

    static uint32_t lastLogMs = 0;
    uint32_t now = millis();
    if (now - lastLogMs >= 1000) {
        lastLogMs = now;
        ESP_LOGI(TAG, "throttle=%.0f%%  pwm=%dµs", throttle * 100.0f, escPwm);
    }
}
