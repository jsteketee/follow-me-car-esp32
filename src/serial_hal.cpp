// serial_hal — 50 Hz USB-CDC JSON telemetry stream for the Pi ROS2 bridge.
// Interleaves with ESP_LOG output; the Pi bridge ignores any line not starting with '{'.
// Day 2: add RX path here to parse {"target_speed":...,"target_angle":...} setpoints.
#include "serial_hal.h"
#include "uwb.h"
#include "imu.h"
#include "rpm.h"
#include "camera.h"
#include "fusion.h"
#include "utils.h"
#include <Arduino.h>

static RateGate _gate{20};  // 50 Hz

// Serial already opened by main.cpp setup(); nothing to do here.
void serial_hal_init() {}

// Emits one JSON frame if the 20 ms gate has elapsed.
void serial_hal_update() {
    float dt;
    if (!_gate.tick(dt)) return;

    const UWBReading& uwb  = uwb_get();
    const ImuData&    imu  = imu_get();
    const RPMData&    rpm  = rpm_get();
    const CameraData& cam  = camera_get();
    const Pose&       pose = fusion_get();

    Serial.printf(
        "{\"ts\":%lu"
        ",\"uwb_dist\":%.1f,\"uwb_bearing\":%.2f"
        ",\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f"
        ",\"speed\":%.3f,\"odo\":%.1f,\"enc_speed\":%.3f,\"cogging\":%d"
        ",\"cam_found\":%d,\"cam_x\":%.3f,\"cam_y\":%.3f"
        ",\"fused_angle\":%.2f,\"fused_dist\":%.1f,\"fused_unc\":%.2f"
        "}\n",
        millis(),
        uwb.distCm, uwb.angleDeg,
        imu.yaw, imu.pitch, imu.roll,
        rpm.speedMph, rpm.odometryCm, rpm.encSpeedMph, (int)rpm.cogging,
        (int)cam.found, cam.posX, cam.posY,
        pose.fusedAngle, pose.distanceCm, pose.uncertainty);
}
