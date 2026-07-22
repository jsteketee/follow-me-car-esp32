#!/usr/bin/env python3
# Live angle + speed plotter for the AS5600 magnetic encoder.
# Requires: pip install pyserial matplotlib
# Usage: python as5600_plot.py [port] [baud]
#   port defaults to auto-detect first /dev/cu.usbmodem* on macOS
#   baud defaults to 115200
#
# Flash the ESP32 with env:as5600-test before running.
# The script sends "mon 200" on connect to start 200 Hz angle streaming.
# Speed is computed identically to rpm.cpp: (delta * RPM_CM_PER_COUNT / dt) * 0.0223694 → mph.

import sys
import glob
import re
import collections
import threading
import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider

PORT   = sys.argv[1] if len(sys.argv) > 1 else None
BAUD   = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
WINDOW = 50  # samples to display (~0.25 seconds at 200 Hz)

# Must match config.h
AS5600_COUNTS_PER_REV    = 4096
AS5600_MIN_MOTION_COUNTS = 2
RPM_CM_PER_COUNT         = -0.000616  # negated vs config.h: test firmware logs RAW_ANGLE (0x0C) which counts opposite to ANGLE (0x0E) used by main firmware
RPM_STALE_MS             = 250       # ms of no motion before speed is zeroed

def auto_detect_port():
    candidates = (glob.glob('/dev/cu.usbmodem*')
                + glob.glob('/dev/cu.SLAB*')
                + glob.glob('/dev/cu.wchusbserial*'))
    if candidates:
        return candidates[0]
    raise RuntimeError("No USB serial port found — pass port as first argument.")

port = PORT or auto_detect_port()
print(f"Connecting to {port} at {BAUD} baud...")

raw_deg         = collections.deque([float('nan')] * WINDOW, maxlen=WINDOW)
filt_deg        = collections.deque([float('nan')] * WINDOW, maxlen=WINDOW)
speeds_mph      = collections.deque([float('nan')] * WINDOW, maxlen=WINDOW)  # EMA-smoothed, derived from RAW angle
speeds_mph_filt = collections.deque([float('nan')] * WINDOW, maxlen=WINDOW)  # EMA-smoothed, derived from FILTERED angle
speeds_mph_hall = collections.deque([float('nan')] * WINDOW, maxlen=WINDOW)  # raw hall-effect speed from firmware
enc_vel_mph     = collections.deque([float('nan')] * WINDOW, maxlen=WINDOW)  # firmware encoder EMA velocity (COG_DIAG)
sign_changes    = collections.deque([float('nan')] * WINDOW, maxlen=WINDOW)  # sign-change count from cogging window (COG_DIAG)

ema_alpha = 0.3   # initial smoothing — slider adjusts this live; shared by both streams so the comparison is apples-to-apples
_ser      = None  # shared serial reference so the throttle slider callback can write commands

# Wall-clock arrival time of every parsed sample — used to measure the actual observed poll rate
# (firmware timestamps only reflect polling cadence, not whether serial reads keep up with it).
poll_times = collections.deque(maxlen=2000)

# Parse the ESP32 uptime timestamp (ms), the RAW angle count, and the filtered ANGLE count.
# Using the firmware timestamp for dt avoids Python scheduling jitter.
PATTERN     = re.compile(r'\[\s*(\d+)\].*RAW=(\d+).*ANGLE=(\d+).*HALL=([\d.]+)')
COG_PATTERN = re.compile(r'COG_DIAG: encVel=([-\d.]+)mph\s+signChanges=(\d+)')

def compute_delta(count, last_count):
    # Shortest-path delta — mirrors the wraparound logic in rpm.cpp.
    delta = count - last_count
    if delta >  AS5600_COUNTS_PER_REV // 2:
        delta -= AS5600_COUNTS_PER_REV
    if delta < -AS5600_COUNTS_PER_REV // 2:
        delta += AS5600_COUNTS_PER_REV
    return delta

def process_stream(count, ts_ms, dt_ms, state):
    # Runs one angle-count stream through delta -> stale-check -> EMA, mirroring rpm.cpp exactly.
    # `state` is a per-stream dict so raw and filtered can be run through this unmodified, in parallel,
    # with the same alpha — the only difference between the two calls is which count sequence goes in.
    if state['last_count'] is None:
        state['last_count']     = count
        state['last_motion_ms'] = ts_ms
        state['ema']            = 0.0
        return 0.0

    delta = compute_delta(count, state['last_count'])
    state['last_count'] = count

    if abs(delta) >= AS5600_MIN_MOTION_COUNTS:
        state['last_motion_ms'] = ts_ms

    stale = (ts_ms - state['last_motion_ms']) > RPM_STALE_MS
    if stale or dt_ms <= 0:
        raw_speed = 0.0
    else:
        dt = dt_ms / 1000.0
        # distance delta → cm/s → mph  (signed: positive = forward)
        dist_delta_cm = delta * RPM_CM_PER_COUNT
        raw_speed     = (dist_delta_cm / dt) * 0.0223694  # cm/s → mph

    # EMA smoothing — alpha read from slider, so no lock needed (float write is atomic in CPython)
    state['ema'] += ema_alpha * (raw_speed - state['ema'])
    return state['ema']

def serial_reader():
    global _ser
    last_ts_ms = None
    state_raw  = {'last_count': None, 'last_motion_ms': None, 'ema': None}
    state_filt = {'last_count': None, 'last_motion_ms': None, 'ema': None}
    hall_ema   = 0.0

    with serial.Serial(port, BAUD, timeout=1) as ser:
        _ser = ser
        time.sleep(0.5)
        ser.write(b'mon 200\n')
        print("Sent: mon 200")
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
            cog_m = COG_PATTERN.search(line)
            if cog_m:
                enc_vel_mph.append(float(cog_m.group(1)))
                sign_changes.append(float(cog_m.group(2)))

            m = PATTERN.search(line)
            if not m:
                continue

            poll_times.append(time.time())

            ts_ms      = int(m.group(1))
            count      = int(m.group(2))
            filt_count = int(m.group(3))
            hall_speed = float(m.group(4))
            deg        = count * 360.0 / AS5600_COUNTS_PER_REV
            filt_deg_v = filt_count * 360.0 / AS5600_COUNTS_PER_REV
            raw_deg.append(deg)
            filt_deg.append(filt_deg_v)

            dt_ms = 0 if last_ts_ms is None else ts_ms - last_ts_ms
            last_ts_ms = ts_ms

            speeds_mph.append(process_stream(count,      ts_ms, dt_ms, state_raw))
            speeds_mph_filt.append(process_stream(filt_count, ts_ms, dt_ms, state_filt))
            hall_ema += ema_alpha * (hall_speed - hall_ema)
            speeds_mph_hall.append(hall_ema)

thread = threading.Thread(target=serial_reader, daemon=True)
thread.start()

fig, (ax_angle, ax_speed, ax_cog) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
fig.subplots_adjust(bottom=0.18, hspace=0.35)   # make room for two sliders
fig.suptitle('AS5600 Encoder — Angle, Speed & Cogging Diagnostics')

line_raw,  = ax_angle.plot([], [], color='tab:blue',  linewidth=1.0, label='raw')
line_filt, = ax_angle.plot([], [], color='tab:green', linewidth=1.0, label='filtered')
ax_angle.set_xlim(0, WINDOW)
ax_angle.set_ylim(-10, 370)
ax_angle.set_yticks([0, 90, 180, 270, 360])
ax_angle.axhline(0,   color='gray', linewidth=0.5, linestyle='--')
ax_angle.axhline(360, color='gray', linewidth=0.5, linestyle='--')
ax_angle.set_ylabel('degrees')
ax_angle.legend(loc='upper right', fontsize=8)

line_speed,      = ax_speed.plot([], [], color='tab:blue',   linewidth=1.0, label='from raw angle (EMA)')
line_speed_filt, = ax_speed.plot([], [], color='tab:green',  linewidth=1.0, label='from filtered angle (EMA)')
line_speed_hall, = ax_speed.plot([], [], color='tab:orange', linewidth=1.0, label='hall-effect (firmware)')
ax_speed.set_xlim(0, WINDOW)
ax_speed.set_ylim(-4.0, 12.0)
ax_speed.axhline(0, color='gray', linewidth=0.5, linestyle='--')
ax_speed.set_ylabel('speed (mph)')
ax_speed.legend(loc='upper right', fontsize=8)

# Cogging diagnostics panel — encoder EMA velocity (left axis) and sign-change count (right axis).
line_enc_vel,    = ax_cog.plot([], [], color='tab:purple', linewidth=1.0, label='enc velocity (mph)')
ax_cog.set_xlim(0, WINDOW)
ax_cog.set_ylim(-3.0, 3.0)
ax_cog.axhline(0, color='gray', linewidth=0.5, linestyle='--')
ax_cog.set_ylabel('enc velocity (mph)', color='tab:purple')
ax_cog.tick_params(axis='y', labelcolor='tab:purple')
ax_cog.set_xlabel(f'samples  (window = {WINDOW} samples @ 200 Hz ≈ {WINDOW//200}s)')

ax_cog_r = ax_cog.twinx()
line_sign_chg,  = ax_cog_r.plot([], [], color='tab:red', linewidth=1.0, alpha=0.7, label='sign changes')
ax_cog_r.set_ylim(0, 25)
ax_cog_r.set_ylabel('sign changes', color='tab:red')
ax_cog_r.tick_params(axis='y', labelcolor='tab:red')

# Combined legend for both cog axes
lines_cog  = [line_enc_vel, line_sign_chg]
labels_cog = [l.get_label() for l in lines_cog]
ax_cog.legend(lines_cog, labels_cog, loc='upper right', fontsize=8)

xs = list(range(WINDOW))

# EMA alpha slider — left=0.01 (heavy smoothing), right=1.0 (no smoothing)
ax_slider = fig.add_axes([0.15, 0.11, 0.70, 0.03])
slider = Slider(ax_slider, 'EMA alpha', 0.01, 1.0, valinit=ema_alpha, valstep=0.01)

def on_slider(val):
    global ema_alpha
    ema_alpha = slider.val
slider.on_changed(on_slider)

# Throttle slider — sends `thr <val>` over serial; zero snaps back to neutral.
ax_thr = fig.add_axes([0.15, 0.04, 0.70, 0.03])
slider_thr = Slider(ax_thr, 'Throttle', 0.0, 0.35, valinit=0.0, valstep=0.01)

def on_throttle(val):
    if _ser and _ser.is_open:
        _ser.write(f'thr {val:.3f}\n'.encode())
slider_thr.on_changed(on_throttle)

def poll_rate_hz():
    # Samples arrived in the last 1.0s of wall-clock time — reflects the actual observed rate,
    # including any serial/USB overhead, not just the firmware's requested polling cadence.
    now = time.time()
    recent = [t for t in poll_times if now - t <= 1.0]
    return float(len(recent))

def update(_):
    rdata  = list(raw_deg)
    fdata  = list(filt_deg)
    sdata  = list(speeds_mph)
    sfdata = list(speeds_mph_filt)
    shdata = list(speeds_mph_hall)
    evdata = list(enc_vel_mph)
    scdata = list(sign_changes)

    line_raw.set_data(xs, rdata)
    line_filt.set_data(xs, fdata)
    line_speed.set_data(xs, sdata)
    line_speed_filt.set_data(xs, sfdata)
    line_speed_hall.set_data(xs, shdata)
    line_enc_vel.set_data(xs, evdata)
    line_sign_chg.set_data(xs, scdata)

    valid_r  = [v for v in rdata  if v == v]
    valid_s  = [v for v in sdata  if v == v]
    valid_sf = [v for v in sfdata if v == v]
    valid_sh = [v for v in shdata if v == v]
    valid_ev = [v for v in evdata if v == v]
    valid_sc = [v for v in scdata if v == v]
    hz = poll_rate_hz()

    if valid_r:
        ax_angle.set_title(f'angle: raw={valid_r[-1]:.1f}°  |  poll rate: {hz:.0f} Hz', fontsize=9)
    if valid_s and valid_sf and valid_sh:
        ax_speed.set_title(f'speed: raw={valid_s[-1]:.3f}mph  filt={valid_sf[-1]:.3f}mph  hall={valid_sh[-1]:.3f}mph', fontsize=9)
    if valid_ev and valid_sc:
        ax_cog.set_title(f'cogging diag: encVel={valid_ev[-1]:.3f}mph  signChanges={int(valid_sc[-1])}', fontsize=9)

    return line_raw, line_filt, line_speed, line_speed_filt, line_speed_hall, line_enc_vel, line_sign_chg

ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
plt.tight_layout()
plt.show()
