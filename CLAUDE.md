# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **Status (2026-07-08):** Parts of this file are **legacy**. The Architecture and File Directory
> sections describe the *pre-DW3000* hardware — three RYUW122 UWB anchors with trilateration and
> hall-effect-only RPM — and predate the current setup (DW3000 AoA anchor giving distance+bearing,
> AS5600 encoder for cogging detection, plus the `serial_hal` and `actuators` modules). Treat those
> sections as historical context, not current truth.
>
> For the **ROS2 / Pi phase** of the build, the docs in the `follow-me-car-ros2` repo
> (`PROJECT_PLAN.md`, `NOTES.md`, `CLAUDE.md`) are authoritative and **supersede this file**.
>
> The **Coding Rules below are current and still apply** — enforce and remember them regardless of
> the legacy architecture notes.

## Coding Rules

- Do not remove comments that appear on their own line. These are intentional notes left by the developer.
- Each module owns a single static state struct. Name it with an underscore prefix in lowerCamelCase after the module (e.g. `_navData`, `_uwbData`, `_imuData`, `_rpmData`) — not the generic `_data` — so the type is clear at every use site without needing to look at the declaration.
- Never include Claude attribution in git commit messages. Do not add "Co-Authored-By" lines or any mention of Claude Code.

## Project Overview

ESP32-S3 firmware for a follow-me RC car. The car uses two UWB (Ultra-Wideband) ranging modules to triangulate a tag's position relative to the car, a third front-facing UWB anchor for front/back disambiguation, and an I2C blob-detection camera — all fused via a Kalman filter on absolute compass bearing — then drives toward it autonomously.

## Build & Flash Commands

```bash
# Build and upload over USB
pio run -e car --target upload

# Monitor serial output (USB)
pio device monitor

# Upload over WiFi (car must be on and reachable)
pio run -e car --target upload --upload-port 192.168.4.1

# Connect to telnet terminal over car's WiFi AP
stty raw -echo && nc 192.168.4.1 23; stty -raw echo
```

The car hosts its own WiFi AP (`Follow Me`, no password). Telnet on port 23 mirrors USB serial and accepts control input. The web dashboard is at `http://192.168.4.1/`.

## Secrets Setup

Copy `include/secrets.example.h` to `include/secrets.h` and fill in WiFi credentials. `secrets.h` is gitignored.

## Architecture

### Data pipeline (runs every loop iteration)

```
imu_update()
uwb_update()
camera_update()         ← optional, only if camera_init() succeeded
fusion_update()         ← blends UWB + camera via Kalman on absolute bearing
nav_update()            ← mode management; reads fusion_get()
control_update()        ← PID throttle + steering; reads nav_get() + fusion_get() + rpm_get()
oled_update()
wifi_update()
rpm_update()
dashboard_update()
```

Each subsystem owns a static struct and exposes it via a `_get()` function. No RTOS — everything runs cooperatively in `loop()`.

### Key data structs and what they represent

- **`UWBReading`** (`uwb.h`) — raw sensor layer. `distLeft`/`distRight` are Kalman-filtered per-anchor distances used for geometry. `distFront` is the Kalman-filtered front anchor distance used for front/back disambiguation. `distFast` is the unfiltered average of left+right used for distance estimation (more responsive). Negative values mean no valid reading.
- **`CameraData`** (`camera.h`) — blob detection result. `found` indicates a blob was detected; `posX`/`posY` are normalized −1.0 to +1.0; `timestamp` is millis() of last poll.
- **`Pose`** (`fusion.h`) — fused navigation layer. `fusedAngle` is Kalman-filtered degrees to tag (0=straight ahead, +right, −left). `uwbAngle`/`camAngle` are the latest raw inputs. `distanceCm` comes from `distFast` with dead-reckoning between UWB updates. `uncertainty` is bearing Kalman variance (low = confident, high = stale).
- **`NavData`** (`nav.h`) — mode and validity. `sensorsValid` is false when `fusion.uncertainty` exceeds `FUSION_STALE_UNCERTAINTY`. `headingHold` is the compass heading captured on mode transition for non-FOLLOW_ME modes.
- **`ControlOutput`** (`control.h`) — throttle and steering, both −1.0 to +1.0.
- **`RPMData`** (`rpm.h`) — speed and odometry. `speedMph` is Kalman-filtered from hall-effect pulse periods. `odometryCm` is cumulative distance since startup.
- **`RuntimeConfig`** (`runtime_config.h`) — mutable copy of all tunable config values; initialized from `config.h` defaults. Modified at runtime via the web dashboard; reset to defaults on reboot.

### UWB ranging

Three RYUW122 modules on Serial0/Serial1/Serial2. The cycle is non-blocking: left → right → front, each polled in sequence using raw AT commands (`AT+ANCHOR_SEND`) and `+ANCHOR_RCV=` responses. The RYUW122 Arduino library's ranging function is broken — the code uses raw AT commands. The library is only used for init and calibration.

Heading is computed via trilateration from `distLeft`/`distRight` and the known physical separation (`UWB_ANCHOR_SEPARATION_CM`). `distFront` resolves the front/back ambiguity via a latched comparison with hysteresis (`UWB_FRONT_FLIP_ABRUPT_DEG` + `UWB_FRONT_FLIP_CONFIRM`).

### Camera

An OV2640 on a XIAO ESP32-S3 runs blob detection and exposes results over I2C at address `0x42`. The car's ESP32-S3 polls it at `CAMERA_UPDATE_INTERVAL_MS`. Camera is optional — `camera_init()` returns false if not found, and `loop()` skips `camera_update()` in that case.

### Sensor fusion

`fusion.cpp` tracks the tag's **absolute compass bearing** in a 1-D Kalman filter. Both UWB (via `calc_tag_heading`) and camera (blob posX × half-FOV) provide relative angle measurements; each is converted to absolute bearing using `imu.yaw` before being fed to the filter. Converting back to relative angle at render time gives automatic rotation compensation without gyro integration. Bearing uncertainty grows every loop; UWB or camera fixes reduce it. Dead reckoning subtracts RPM odometry from `distanceCm` between UWB updates.

### Kalman filter tuning

`UWB_KALMAN_Q` (process noise) and `UWB_KALMAN_R` (measurement noise) are in `include/config.h`. Raising Q makes the per-anchor filter track faster. Fusion R values control how much each sensor type corrects the fused bearing per update — lower R = more trust. All tunable at runtime via dashboard sliders (stored in `rtConfig`).

### PID control

Throttle is speed-PID: setpoint = `targetSpeedMph`, measurement = `rpm.speedMph`. Steering is angle-PID: setpoint = 0°, measurement = `fusedAngle`. Feed-forward (`throttleFfK`) and exponential smoothing (`smoothAlpha`) are applied to throttle output. Both PID loops run at `CONTROL_UPDATE_INTERVAL_MS` (50 Hz).

### Web dashboard

Served at `http://192.168.4.1/` (port 80). Pushes JSON telemetry via WebSocket at ~10 Hz. Accepts `POST /config?key=...&value=...` to update `rtConfig` fields and `POST /mode?mode=...` to change nav mode. All tunable parameters have live sliders.

### Logging

Use ESP-IDF log macros throughout (not `Serial.print`):

| Level | Macro | When |
|-------|-------|------|
| Error | `ESP_LOGE` | Failed, can't continue |
| Warn | `ESP_LOGW` | Unexpected but recoverable |
| Info | `ESP_LOGI` | Normal milestones |
| Debug | `ESP_LOGD` | Detailed state during development |

`CORE_DEBUG_LEVEL=3` (Info) is set in all build environments.

### Pin assignments

All pins are in `include/config.h`. UWB modules use three dedicated UARTs (Serial0/1/2). IMU (BNO085) is on I2C at address `0x4B`. Camera is on the same I2C bus at `0x42`. ESC and steering servo use PWM (1000–2000µs, neutral 1500µs).

## File Directory

| File | Purpose |
|------|---------|
| `src/main.cpp` | Arduino `setup()`/`loop()`. Initializes all subsystems, runs the cooperative main loop, and reports per-module timing. |
| `src/uwb.cpp/.h` | UWB ranging driver. Non-blocking cycle: polls left → right → front RYUW122 anchors via raw AT commands, applies per-anchor Kalman filtering and outlier rejection. Exposes `UWBReading`. |
| `src/camera.cpp/.h` | I2C blob-detection camera driver. Polls XIAO at `CAMERA_I2C_ADDR` every `CAMERA_UPDATE_INTERVAL_MS`; exposes `CameraData`. Optional — `camera_init()` returns false if not present. |
| `src/fusion.cpp/.h` | Sensor fusion layer. Maintains absolute compass bearing estimate via 1-D Kalman filter fed by UWB trilateration and camera blob angle. Dead-reckons distance from RPM odometry. Exposes `Pose`. |
| `src/nav.cpp/.h` | Navigation mode management. Reads `fusion_get()` to set `sensorsValid`; manages mode transitions and `headingHold`. Exposes `NavData`. |
| `src/control.cpp/.h` | ESC + steering servo output. Speed-PID on `rpm.speedMph`, angle-PID on `fusion.fusedAngle`. Applies steering trim. Exposes `ControlOutput`. |
| `src/imu.cpp/.h` | BNO085 IMU driver over I2C. Publishes yaw/pitch/roll, accel, gyro, and calibration status. |
| `src/rpm.cpp/.h` | Hall-effect speed sensor. Interrupt-driven pulse period measurement, Kalman-filtered speed, spike rejection, cumulative odometry. Exposes `RPMData`. |
| `src/oled.cpp/.h` | SSD1306 128×64 OLED display. Renders IMU stats, nav state, throttle/steering, and a heading arrow graphic. |
| `src/dashboard.cpp/.h` | HTTP + WebSocket web dashboard (port 80). Pushes telemetry JSON at ~10 Hz; accepts config and mode POST endpoints. |
| `src/wifi_config.cpp/.h` | Soft-AP WiFi + single-client telnet server (port 23). Mirrors USB serial and forwards input bytes for remote control. |
| `src/runtime_config.cpp/.h` | Mutable runtime copy of all tunable config values (`RuntimeConfig rtConfig`). Initialized from `config.h` defaults; updated by dashboard. |
| `src/utils.h` | Shared inline utilities: `KalmanFilter` (1-D scalar), `AngleKalman` (2-state angle+bias), `PerfTracker`, `RateGate`, `PidController`, `HzTracker`. |
| `include/config.h` | All compile-time constants: pin assignments, timing intervals, Kalman Q/R, throttle/steering PID gains, UWB geometry, fusion parameters. Primary tuning file. |
| `include/secrets.h` | WiFi credentials — gitignored, not in repo. |
| `include/secrets.example.h` | Template to create `secrets.h` from. |
| `platformio.ini` | PlatformIO config. Three environments: `car` (normal), `test`, `debug` — all identical currently. |
| `scripts/uwb_poll.py` | Python script to directly poll a UWB module over USB serial and print distance + rolling average. Useful for bench testing anchors. |
| `scripts/uwb_terminal.py` | Interactive Python terminal for sending raw AT commands to a UWB module over USB serial. |
| `scripts/common commands.txt` | Quick-reference for common PlatformIO and telnet commands. |
| `Project Resources/` | Reference docs (AT command spec PDF, feature spec). Not compiled — context for development only. |
