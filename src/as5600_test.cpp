// Bench validation + interactive I2C console for the AS5600 magnetic encoder module.
// Standalone entry point (env:as5600-test) — does not touch the car's main.cpp or existing rpm.cpp driver.
//
// Two modes running together:
//   1. Periodic monitor (default on at 4 Hz): logs decoded STATUS/AGC/MAGNITUDE/angle each tick so
//      magnet placement can be dialed in by hand. `mon <hz>` changes the rate, `mon 0` silences it.
//   2. Command console over USB serial: newline-terminated commands, responses printed back —
//      usable interactively via `pio device monitor` or driven by a pyserial script.
//
// Commands:
//   scan            — I2C bus scan (prints every ACKing address)
//   status          — one-shot decoded health readout (magnet flags, AGC, magnitude, angle)
//   angle           — one-shot RAW_ANGLE + filtered ANGLE read, in counts and degrees
//   r <reg> [n]     — read n bytes (default 1) starting at hex register <reg>
//   w <reg> <val>   — write one byte <val> to hex register <reg>
//   mon <hz>        — set monitor rate in Hz (0 = off)
#include <Arduino.h>
#include <Wire.h>
#include "esp_log.h"

static const char* TAG = "as5600_test";

// AS5600 sits on the car's main I2C bus (shared with OLED + IMU; pull-ups provided by those breakouts).
static const int SDA_PIN = 8;
static const int SCL_PIN = 9;

static const uint8_t AS5600_ADDR = 0x36;

// AS5600 register map (per datasheet). RAW_ANGLE is the unfiltered reading; ANGLE applies the chip's hysteresis/scaling.
static const uint8_t REG_ZMCO      = 0x00;
static const uint8_t REG_STATUS    = 0x0B;
static const uint8_t REG_RAW_ANGLE = 0x0C; // 0x0C high byte, 0x0D low byte (12-bit)
static const uint8_t REG_ANGLE     = 0x0E; // 0x0E high byte, 0x0F low byte (12-bit)
static const uint8_t REG_AGC       = 0x1A;
static const uint8_t REG_MAGNITUDE = 0x1B; // 0x1B high byte, 0x1C low byte (12-bit)

// STATUS register bits
static const uint8_t STATUS_MH = 1 << 3; // magnet too strong (AGC at minimum)
static const uint8_t STATUS_ML = 1 << 4; // magnet too weak (AGC at maximum)
static const uint8_t STATUS_MD = 1 << 5; // magnet detected

static float    _monitorHz     = 4.0f;
static uint32_t _lastMonitorMs = 0;

static char _cmdBuf[64];
static int  _cmdLen = 0;

// Reads one byte from a register; returns -1 on any I2C error so callers can distinguish failure from data.
static int as5600_read8(uint8_t reg) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom(AS5600_ADDR, (uint8_t)1) != 1) return -1;
    return Wire.read();
}

// Reads a 12-bit big-endian value from a register pair; returns -1 on any I2C error.
static int as5600_read12(uint8_t regHigh) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(regHigh);
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return -1;
    int hi = Wire.read();
    int lo = Wire.read();
    return ((hi & 0x0F) << 8) | lo;
}

// Writes one byte to a register; returns true on ACK.
static bool as5600_write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

// Converts a 12-bit angle count to degrees.
static float counts_to_deg(int counts) {
    return counts * (360.0f / 4096.0f);
}

// Prints every address that ACKs on the bus — confirms the AS5600 (0x36) is visible alongside OLED/IMU.
static void cmd_scan() {
    Serial.println("scanning I2C bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  found device at 0x%02X%s\n", addr, addr == AS5600_ADDR ? "  <- AS5600" : "");
            found++;
        }
    }
    Serial.printf("scan complete, %d device(s)\n", found);
}

// Prints a full decoded health readout: magnet flags, AGC, CORDIC magnitude, and both angle registers.
static void cmd_status() {
    int status = as5600_read8(REG_STATUS);
    if (status < 0) { Serial.println("ERROR: no response from AS5600 (0x36) — check wiring/power"); return; }

    int agc  = as5600_read8(REG_AGC);
    int mag  = as5600_read12(REG_MAGNITUDE);
    int raw  = as5600_read12(REG_RAW_ANGLE);
    int ang  = as5600_read12(REG_ANGLE);

    bool md = status & STATUS_MD;
    bool ml = status & STATUS_ML;
    bool mh = status & STATUS_MH;

    const char* verdict = (!md) ? "NO MAGNET DETECTED"
                        : (ml)  ? "magnet TOO WEAK (increase strength / reduce gap)"
                        : (mh)  ? "magnet TOO STRONG (increase gap)"
                        :         "magnet OK";

    Serial.printf("STATUS=0x%02X  MD=%d ML=%d MH=%d  -> %s\n", status, md, ml, mh, verdict);
    Serial.printf("AGC=%d (target ~mid-range; near 0 = too strong, near max = too weak)\n", agc);
    Serial.printf("MAGNITUDE=%d\n", mag);
    Serial.printf("RAW_ANGLE=%d (%.2f deg)   ANGLE=%d (%.2f deg)\n", raw, counts_to_deg(raw), ang, counts_to_deg(ang));
}

// Prints a one-shot read of both angle registers.
static void cmd_angle() {
    int raw = as5600_read12(REG_RAW_ANGLE);
    int ang = as5600_read12(REG_ANGLE);
    if (raw < 0 || ang < 0) { Serial.println("ERROR: read failed"); return; }
    Serial.printf("RAW_ANGLE=%d (%.2f deg)   ANGLE=%d (%.2f deg)\n", raw, counts_to_deg(raw), ang, counts_to_deg(ang));
}

// Executes one parsed console command line.
static void handle_command(char* line) {
    // strip leading whitespace
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    if (strncmp(line, "scan", 4) == 0)   { cmd_scan();   return; }
    if (strncmp(line, "status", 6) == 0) { cmd_status(); return; }
    if (strncmp(line, "angle", 5) == 0)  { cmd_angle();  return; }

    if (strncmp(line, "mon", 3) == 0) {
        float hz = atof(line + 3);
        _monitorHz = hz;
        Serial.printf("monitor rate set to %.1f Hz%s\n", hz, hz <= 0 ? " (off)" : "");
        return;
    }

    if (line[0] == 'r' && (line[1] == ' ' || line[1] == '\t')) {
        unsigned int reg = 0; int n = 1;
        int parsed = sscanf(line + 1, "%x %d", &reg, &n);
        if (parsed < 1 || reg > 0xFF) { Serial.println("usage: r <hexreg> [count]"); return; }
        if (n < 1) n = 1;
        if (n > 16) n = 16;
        Serial.printf("read 0x%02X x%d:", reg, n);
        for (int i = 0; i < n; i++) {
            int v = as5600_read8((uint8_t)(reg + i));
            if (v < 0) { Serial.print(" ERR"); break; }
            Serial.printf(" %02X", v);
        }
        Serial.println();
        return;
    }

    if (line[0] == 'w' && (line[1] == ' ' || line[1] == '\t')) {
        unsigned int reg = 0, val = 0;
        if (sscanf(line + 1, "%x %x", &reg, &val) != 2 || reg > 0xFF || val > 0xFF) {
            Serial.println("usage: w <hexreg> <hexval>");
            return;
        }
        bool ok = as5600_write8((uint8_t)reg, (uint8_t)val);
        Serial.printf("write 0x%02X = 0x%02X: %s\n", reg, val, ok ? "ok" : "ERR");
        return;
    }

    Serial.printf("unknown command: %s  (commands: scan, status, angle, r, w, mon)\n", line);
}

// Accumulates USB serial input into a line buffer and dispatches completed commands.
static void poll_console() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (_cmdLen > 0) {
                _cmdBuf[_cmdLen] = '\0';
                handle_command(_cmdBuf);
                _cmdLen = 0;
            }
        } else if (_cmdLen < (int)sizeof(_cmdBuf) - 1) {
            _cmdBuf[_cmdLen++] = c;
        }
    }
}

// Emits one compact monitor line (flags, AGC, magnitude, angle) at the configured rate.
static void poll_monitor() {
    if (_monitorHz <= 0.0f) return;
    uint32_t intervalMs = (uint32_t)(1000.0f / _monitorHz);
    if (millis() - _lastMonitorMs < intervalMs) return;
    _lastMonitorMs = millis();

    int status = as5600_read8(REG_STATUS);
    if (status < 0) {
        ESP_LOGW(TAG, "MON: no response from AS5600 (0x36)");
        return;
    }
    int agc = as5600_read8(REG_AGC);
    int mag = as5600_read12(REG_MAGNITUDE);
    int raw = as5600_read12(REG_RAW_ANGLE);

    ESP_LOGI(TAG, "MON: MD=%d ML=%d MH=%d  AGC=%d  MAG=%d  RAW=%d (%.2f deg)",
        (status & STATUS_MD) != 0, (status & STATUS_ML) != 0, (status & STATUS_MH) != 0,
        agc, mag, raw, counts_to_deg(raw));
}

void setup() {
    delay(3000);
    Serial.begin(115200);
    ESP_LOGI(TAG, "======================================================");
    ESP_LOGI(TAG, "AS5600 bench test starting");
    ESP_LOGI(TAG, "Step 1: opening I2C bus SDA=GPIO%d SCL=GPIO%d @400kHz", SDA_PIN, SCL_PIN);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    ESP_LOGI(TAG, "Step 2: probing for AS5600 at 0x%02X", AS5600_ADDR);
    Wire.beginTransmission(AS5600_ADDR);
    if (Wire.endTransmission() == 0) {
        ESP_LOGI(TAG, "AS5600 responded — running initial status readout");
        cmd_status();
    } else {
        ESP_LOGW(TAG, "AS5600 NOT found at 0x%02X — check wiring; `scan` lists visible devices", AS5600_ADDR);
    }

    ESP_LOGI(TAG, "Step 3: console ready (commands: scan, status, angle, r, w, mon) — monitor at %.1f Hz", _monitorHz);
    ESP_LOGI(TAG, "======================================================");
}

void loop() {
    poll_console();
    poll_monitor();
}
