# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Coding Rules

- Do not remove comments that appear on their own line. These are intentional notes left by the developer.

## Project Overview

ESP32-S3 firmware for a follow-me RC car. The car uses two UWB (Ultra-Wideband) ranging modules to triangulate a tag's position relative to the car, then drives toward it autonomously.

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

The car hosts its own WiFi AP (`followme-car`, no password). Telnet on port 23 mirrors USB serial and accepts control input.

## Secrets Setup

Copy `include/secrets.example.h` to `include/secrets.h` and fill in WiFi credentials. `secrets.h` is gitignored.

## Architecture

### Data pipeline (runs every loop iteration)

```
uwb_update() → nav_update() → control_update(nav)
                    ↓
              oled_update(nav)
```

Each subsystem owns a static struct and exposes it via a `_get()` function. No RTOS — everything runs cooperatively in `loop()`.

### Key data structs and what they represent

- **`UWBReading`** (`uwb.h`) — raw sensor layer. `distLeft`/`distRight` are Kalman-filtered per-anchor distances used for geometry. `distFast` is the unfiltered average of both anchors used for distance control (more responsive). `validLeft`/`validRight` indicate whether the last poll succeeded.
- **`NavData`** (`nav.h`) — fused navigation layer. `relativeAngle` is degrees to tag (0=straight ahead, +right, -left). `distanceCm` is populated from `uwb.distFast`. `state` is `VALID` or `STALE`.
- **`ControlOutput`** (`control.h`) — throttle and steering, both -1.0 to 1.0.

### UWB ranging

Two RYUW122 modules on Serial1/Serial2, staggered by `UWB_ANCHOR_STAGGER_MS` to avoid RF collision. The RYUW122 Arduino library's ranging function is broken — the code uses raw AT commands (`AT+ANCHOR_SEND`) and parses `+ANCHOR_RCV=` responses directly. The library is only used for init and calibration.

Heading is computed via trilateration from the two anchor distances and their known physical separation (`UWB_ANCHOR_SEPARATION_CM`).

### Kalman filter tuning

`UWB_KALMAN_Q` (process noise) and `UWB_KALMAN_R` (measurement noise) are in `include/config.h`. The ratio Q/R controls responsiveness vs smoothness. Raising Q makes the filter track faster. Since heading is derived from the *difference* between two filtered values, lag compounds — raise Q to improve angle responsiveness.

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

All pins are in `include/config.h`. UWB modules use three dedicated UARTs. IMU (BNO085) is on I2C at address `0x4B`. ESC and steering servo use PWM (1000–2000µs, neutral 1500µs).

## File Directory

| File | Purpose |
|------|---------|
| `src/main.cpp` | Arduino `setup()`/`loop()`. Initializes all subsystems and runs the cooperative main loop. |
| `src/uwb.cpp/.h` | UWB ranging driver. Polls two RYUW122 anchors via raw AT commands, applies per-anchor Kalman filtering, and exposes `UWBReading`. |
| `src/nav.cpp/.h` | Navigation layer. Trilateration from `distLeft`/`distRight` to compute `relativeAngle`; `distanceCm` comes from `distFast`. Owns `NavState` (VALID/STALE). |
| `src/control.cpp/.h` | ESC + steering servo output. `control_update()` maps `NavData` to throttle/steering PWM. Also contains manual arrow-key control for testing. |
| `src/imu.cpp/.h` | BNO085 IMU driver over I2C. Publishes yaw/pitch/roll, accel, gyro, and calibration status. |
| `src/oled.cpp/.h` | SSD1306 128×64 OLED display. Renders IMU stats, nav state, throttle/steering, and a heading arrow graphic. |
| `src/wifi_config.cpp/.h` | Soft-AP WiFi + single-client telnet server (port 23). Mirrors USB serial and forwards input bytes for remote control. |
| `src/utils.h` | Shared inline utilities: `KalmanFilter` (1-D scalar) and `HzTracker` (rolling rate counter). |
| `include/config.h` | All compile-time constants: pin assignments, timing intervals, Kalman Q/R, throttle scale, UWB geometry. Primary tuning file. |
| `include/secrets.h` | WiFi credentials — gitignored, not in repo. |
| `include/secrets.example.h` | Template to create `secrets.h` from. |
| `platformio.ini` | PlatformIO config. Three environments: `car` (normal), `test`, `debug` — all identical currently. |
| `scripts/uwb_poll.py` | Python script to directly poll a UWB module over USB serial and print distance + rolling average. Useful for bench testing anchors. |
| `scripts/uwb_terminal.py` | Interactive Python terminal for sending raw AT commands to a UWB module over USB serial. |
| `scripts/common commands.txt` | Quick-reference for common PlatformIO and telnet commands. |
| `AI Chat Resources/` | Reference docs (AT command spec PDF, feature spec). Not compiled — context for development only. |
