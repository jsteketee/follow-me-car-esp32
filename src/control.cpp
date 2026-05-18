// ESC and steering servo control. Maps NavData to throttle/steering PWM output via ESP32Servo.
#include "control.h"
#include "config.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include "esp_log.h"
#include "imu.h"

static const char *TAG = "control";

#define PWM_NEUTRAL_US 1500
#define PWM_MIN_US     1000
#define PWM_MAX_US     2000



static Servo escServo;
static Servo steerServo;
static ControlOutput _output = {0.0f, 0.0f};

const ControlOutput& control_get() { return _output; }

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
    _output = out;
    escServo.writeMicroseconds(constrain(float_to_pwm(out.throttle),  PWM_MIN_US, PWM_MAX_US));
    steerServo.writeMicroseconds(constrain(float_to_pwm(out.steering), PWM_MIN_US, PWM_MAX_US));
}

void control_update(const NavData &nav, const ImuData &imu) {
    ControlOutput control = _output;
    digitalWrite(PIN_LED, nav.state == NavState::VALID ? HIGH : LOW);

    //Todo Need to integrate heading change and use that to adjust steering between waypoint measurements.
    if (nav.state == NavState::VALID) {
        control.steering = nav.relativeAngle / 90.0f * -1.0f;
        float d = nav.distanceCm;
        if (d <= FOLLOW_DISTANCE_CM) {
            control.throttle = 0.0f;
        } else {
            float t = constrain((d - FOLLOW_DISTANCE_CM) / (MAX_DISTANCE_CM - FOLLOW_DISTANCE_CM), 0.0f, 1.0f);
            control.throttle = THROTTLE_Deadband + t * (THROTTLE_SCALE - THROTTLE_Deadband);
        }
    } 
    else{
        control.throttle = 0.0f; // stop if no valid nav data
    }
    control_apply(control);
}

