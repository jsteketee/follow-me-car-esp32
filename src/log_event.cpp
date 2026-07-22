// Typed event-frame emitter for the Pi ROS2 bridge. Formats one whole newline-terminated
// JSON line and writes it in a single Serial.write so it never interleaves a telemetry frame.
#include "log_event.h"
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// Maps a level to its wire token (contract: debug|info|warn|error|fatal).
static const char* level_str(LogLevel level) {
    switch (level) {
        case LOG_DEBUG: return "debug";
        case LOG_INFO:  return "info";
        case LOG_WARN:  return "warn";
        case LOG_ERROR: return "error";
        case LOG_FATAL: return "fatal";
    }
    return "info";
}

// Copies src into dst escaping " and \ and dropping control chars, so free text can't break the JSON.
static void json_escape(const char* src, char* dst, size_t cap) {
    size_t n = 0;
    for (; *src && n < cap - 1; src++) {
        char c = *src;
        if (c == '"' || c == '\\') {
            if (n >= cap - 2) break;
            dst[n++] = '\\';
            dst[n++] = c;
        } else if ((uint8_t)c >= 0x20) {
            dst[n++] = c;
        }
    }
    dst[n] = '\0';
}

// Formats and writes the complete frame in one call (atomic against telemetry on the same task).
// Dropped wholesale if the TX buffer can't hold the whole line, so it never blocks or tears a
// telemetry frame — telemetry and control always win.
static void emit(LogLevel level, const char* msg) {
    char esc[LOG_EVENT_MSG_MAX * 2];
    json_escape(msg, esc, sizeof(esc));
    char line[LOG_EVENT_MSG_MAX * 2 + 48];
    int n = snprintf(line, sizeof(line), "{\"type\":\"log\",\"level\":\"%s\",\"msg\":\"%s\"}\n",
                     level_str(level), esc);
    if (n <= 0) return;
    size_t len = (n < (int)sizeof(line)) ? (size_t)n : sizeof(line) - 1;
    if (Serial.availableForWrite() < (int)len) return;  // full TX buffer: drop, never block
    Serial.write((const uint8_t*)line, len);
}

void log_event(LogLevel level, const char* fmt, ...) {
    if (level < LOG_EVENT_MIN_LEVEL) return;
    char msg[LOG_EVENT_MSG_MAX];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    emit(level, msg);
}

// Per-key throttle table. Keys are stable string literals, compared by content; the set of
// distinct keys is small (one per fault site), so a short linear table is enough.
#define LOG_EVENT_THROTTLE_SLOTS 8
struct ThrottleSlot { const char* key; uint32_t lastMs; };
static ThrottleSlot _throttle[LOG_EVENT_THROTTLE_SLOTS] = {};

void log_event_throttled(const char* key, LogLevel level, uint32_t minIntervalMs, const char* fmt, ...) {
    if (level < LOG_EVENT_MIN_LEVEL) return;
    uint32_t now = millis();

    ThrottleSlot* slot = nullptr;
    for (int i = 0; i < LOG_EVENT_THROTTLE_SLOTS; i++)
        if (_throttle[i].key && strcmp(_throttle[i].key, key) == 0) { slot = &_throttle[i]; break; }

    if (slot) {
        if (now - slot->lastMs < minIntervalMs) return;  // within the throttle window — drop
    } else {
        for (int i = 0; i < LOG_EVENT_THROTTLE_SLOTS; i++)
            if (!_throttle[i].key) { slot = &_throttle[i]; break; }
        if (!slot) slot = &_throttle[0];  // table full: evict the oldest-registered slot
    }
    slot->key    = key;
    slot->lastMs = now;

    char msg[LOG_EVENT_MSG_MAX];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    emit(level, msg);
}
