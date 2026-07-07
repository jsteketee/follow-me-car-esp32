// Hardware actuation layer for ESC and steering servo.
// Translates normalized [-1, 1] commands from control.cpp into PWM signals,
// applying deadband, scale, EMA smoothing, steering trim, and output clamping.
// All code that physically moves the car passes through this module.
#pragma once

// Attaches ESC and steering servo and arms both to neutral PWM.
void actuators_init();

// Accepts [-1, 1] throttle and steering, applies all hardware transforms, and writes to servos.
void actuators_set(float throttle, float steering);
