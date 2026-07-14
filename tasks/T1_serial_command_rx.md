# T1 — Serial command RX + Pi-commanded nav mode + cmd-timeout failsafe

> **STATUS: COMPLETE + bench-validated 2026-07-13.** Historical brief — parts were
> deliberately superseded the same day at the user's direction: failsafe is now
> throttle-only (steering holds last heading, not centered), `DEFAULT_NAV_MODE` is REMOTE,
> FOLLOW_ME's control block is commented out, and the telemetry frame changed (camera
> fields removed; `lax` + command echo + actuator outputs added). Current truth:
> `../follow-me-car-ros2/PROJECT_PLAN.md` and `tasks/HANDOFF_2026-07-13.md`.

Task brief for an implementation agent. Self-contained, but the authoritative spec is
`../follow-me-car-ros2/PROJECT_PLAN.md` — read its "Serial Protocol (ESP32 ↔ Pi)" and
"Control Loop Placement" sections before writing code. Repo conventions are in this repo's
`CLAUDE.md` (one static state struct per module, exposed via `<mod>_get()`).

## Context

- Repo: `follow-me-car-esp32` (PlatformIO, ESP32-S3, Arduino framework). Work on branch
  `ros2-hal`. Build/verify with `pio run -e car`.
- The firmware currently streams 50 Hz JSON telemetry to a Raspberry Pi over native USB-CDC
  (`src/serial_hal.cpp`) but has **zero inbound serial handling**. This task adds the
  command path down.
- The main loop is cooperative (no RTOS tasks of ours); modules self-gate with `RateGate`
  (`src/utils.h`). The 50 Hz PIDs run in `src/control.cpp`. **Nothing in this task may
  block the loop.**

## Scope — exactly four things

### 1. Non-blocking serial RX parser (`src/serial_hal.cpp`)

- In `serial_hal_update()`, drain `while (Serial.available())` byte-by-byte into a static
  line buffer (~128 B); process a frame only when `'\n'` arrives. On buffer overflow,
  discard bytes until the next `'\n'`. Resync on `'{'`.
- **Forbidden:** `readStringUntil`, `parseFloat`, `parseInt`, or anything that blocks on the
  Stream timeout.
- Parse frames of the form `{"target_speed":1.8,"target_heading":214.5}`. A strict
  hand-rolled key scanner is fine (match the hand-rolled printf TX style); if you prefer
  ArduinoJson, add it to `lib_deps` and do NOT enable `ARDUINOJSON_ENABLE_NAN`.
- **Validation is mandatory, not optional** (a NaN reaching the actuator smoothing EMA
  poisons it permanently and rails PWM — see `actuators.cpp` `float_to_pwm`):
  - Reject the whole frame if either value is non-finite (`isfinite()`), or if
    `target_speed` is outside `[0, rtConfig.maxSpeedMph]` (no reverse), or if
    `target_heading` is outside `[-360, 720]` (then normalize to `[0, 360)`).
  - Silently drop lines that aren't valid command frames (logs, garbage, torn lines).
- Store the accepted command in a module-owned struct per repo convention, e.g.
  `struct CommandData { float targetSpeedMph; float targetHeadingDeg; uint32_t lastCmdMs; }`
  exposed via `serial_hal_get()`. Stamp `lastCmdMs = millis()` **only when a frame parses
  AND validates**.

### 2. New nav mode `NavMode::REMOTE` (`src/nav.h`, `src/control.cpp`, `src/dashboard.cpp`)

- Add `REMOTE` to the `NavMode` enum and a case to `control.cpp`'s `switch (nav.mode)`:
  - Speed: `targetSpeed = cmd.targetSpeedMph` (existing speed PID unchanged).
  - Steering: reuse the existing steering PID with **setpoint 0 and measure = heading error
    wrapped to [-180, 180]**: `err = imu.yaw − cmd.targetHeadingDeg`, wrapped with the same
    while-loop pattern used in `fusion.cpp`'s `correct_bearing()`. This mirrors FOLLOW_ME
    exactly (whose measure `fusedAngle` is `yaw − bearing`, wrapped) so the tuned gains
    transfer — do not retune anything.
  - **Wrap correctness matters**: a target on the far side of the 0/360 seam must produce
    the short-way error (e.g. yaw=10, target=350 → err=+20, not −340).
- Add `REMOTE` to the dashboard `/mode` handler (`dashboard.cpp`) and to `nav_mode_str`.
- **Mode authority rule:** serial frames NEVER change the nav mode. Frames arriving outside
  REMOTE are ignored (parser may still run; just don't act on the values). Dashboard
  `/mode` — including STOPPED — always wins.

### 3. Cmd-timeout failsafe (`src/control.cpp`)

- In `control_update()`, when `nav.mode == REMOTE` and
  `millis() − cmd.lastCmdMs > 300`: force the STOPPED-equivalent path — both setpoints NaN
  (PIDs reset), throttle 0 AND steering 0 (neutral both axes).
- The car **stays in REMOTE** and auto-resumes when the next valid frame arrives (hybrid
  semantics per PROJECT_PLAN "Command stream contract"). The timeout constant lives in
  `include/config.h` (e.g. `CMD_TIMEOUT_MS = 300`), not in rtConfig.
- The timeout check lives in `control_update()` (which is the actuator-owning path), not in
  the parser.

### 4. Nothing else

Explicitly OUT of scope (do not touch): telemetry frame contents, `DEFAULT_NAV_MODE`,
task watchdog, `fusion.cpp` / `nav.cpp` behavior, camera, dashboard beyond the `/mode`
addition, WiFi/OTA, stripping any existing code. Do not remove existing comments; follow
the comment style in the repo (file headers + one-line function descriptions).

## Acceptance criteria

1. `pio run -e car` builds clean.
2. Bench (car on a stand, wheels off the ground, dashboard open as kill switch):
   - In REMOTE via dashboard `/mode`, typing
     `{"target_speed":1.0,"target_heading":<current yaw>}` into `pio device monitor`
     spins the wheels and holds steering near center; changing `target_heading` by ±30°
     deflects steering the correct direction.
   - Stop typing: within ~300 ms wheels return to neutral, steering centers. Typing another
     valid frame resumes without any mode change.
   - `{"target_speed":NaN,"target_heading":0}` and `{"target_speed":-1,...}` are rejected —
     no state change, no NaN ever reaches actuators.
   - Seam test: with yaw near 0°, `target_heading:350` steers the short way.
   - Dashboard STOPPED while frames are streaming: car stops and stays stopped.
3. FOLLOW_ME / TEST / THROTTLE_TEST / STOPPED behavior is unchanged.
4. Loop rate is not measurably degraded (the parser drains a 256 B ring; it must cost
   microseconds, not milliseconds).

## Report back

List files changed, any deviation from this brief with justification, and the observed
results of each acceptance test. Do not commit or push.
