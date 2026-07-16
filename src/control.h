#pragma once

// Control modes for the HAL-era firmware: the ESP32 no longer owns nav logic. The
// dashboard /mode endpoint is the sole mode authority — serial frames never change it.
enum class ControlMode {
    SETPOINT,   // Pi commands setpoints (target_speed/target_heading); onboard PIDs close the loops
    DIRECT,   // Pi commands raw actuator efforts (throttle/steering, normalized); PIDs bypassed
    STOPPED,  // kill switch: both axes neutral; latches until a human re-arms via the dashboard
};

struct ControlOutput {
    float throttle;        // -1.0 (full reverse) to 1.0 (full forward), 0.0 = neutral
    float steering;        // -1.0 (full left) to 1.0 (full right), 0.0 = center
    float targetSpeedMph;  // interpolated speed setpoint fed to PID this cycle; 0 when stopped
};

void control_init();
bool control_update();  // true when the PID rate gate fired and outputs were updated
const ControlOutput& control_get();
void        control_set_mode(ControlMode m);
ControlMode control_mode();
const char* control_mode_str(ControlMode m);  // human-readable mode name, shared by logs and telemetry
float control_setpoint_heading_deg();  // SETPOINT held target heading (boot-seeded, updated by valid commands); NAN until seeded
