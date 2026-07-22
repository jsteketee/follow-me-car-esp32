// Typed event frames on the telemetry serial stream (contract: follow-me-car-ros2
// PROJECT_PLAN.md "Typed event frames"). Emits {"type":"log","level":..,"msg":..} lines
// the Pi bridge re-logs to /rosout; regular telemetry frames carry no "type" key.
#pragma once
#include <stdint.h>

enum LogLevel { LOG_DEBUG = 0, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_FATAL };

// Levels below this compile the call to nothing — the channel is for warn+ anomalies
// worth a human's attention while driving (the Pi dashboard filters below warn anyway).
#define LOG_EVENT_MIN_LEVEL LOG_WARN
#define LOG_EVENT_MSG_MAX   96   // max msg length before truncation (single dashboard line)

// Emits one event frame. MAIN-LOOP ONLY — never call from an ISR, the OLED render task, or
// a web-server handler: a mid-line preemption of a telemetry write tears both frames.
void log_event(LogLevel level, const char* fmt, ...);

// Same, but per-key throttled: identical `key` emits at most once per minIntervalMs, so a
// fault re-tripping at loop rate can't stream. `key` must be a stable string literal.
void log_event_throttled(const char* key, LogLevel level, uint32_t minIntervalMs, const char* fmt, ...);
