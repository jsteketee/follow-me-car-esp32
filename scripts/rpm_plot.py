#!/usr/bin/env python3
# Live serial plotter for hall-effect speed, encoder EMA velocity, and cogging detection.
# Requires: pip install pyserial matplotlib
# Usage: python3 rpm_plot.py [port] [baud]
#   port defaults to auto-detect first /dev/cu.usbmodem* on macOS
#   baud defaults to 115200
#
# Flash the ESP32 with env:car before running.

import sys
import glob
import re
import time
import collections
import threading
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

PORT   = sys.argv[1] if len(sys.argv) > 1 else None
BAUD   = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
WINDOW = 500                # samples (~2.5 seconds at 200 Hz)
RPM_COGGING_WINDOW_MAX = 21 # must match RPM_COGGING_WINDOW in config.h (CYCLE_SAMPLES * 3)

def auto_detect_port():
    candidates = (glob.glob('/dev/cu.usbmodem*')
                + glob.glob('/dev/cu.SLAB*')
                + glob.glob('/dev/cu.wchusbserial*'))
    if candidates:
        return candidates[0]
    raise RuntimeError("No USB serial port found — pass port as first argument.")

port = PORT or auto_detect_port()
print(f"Connecting to {port} at {BAUD} baud...")

speed_buf    = collections.deque([float('nan')] * WINDOW, maxlen=WINDOW)
enc_vel_buf  = collections.deque([float('nan')] * WINDOW, maxlen=WINDOW)
cogging_buf  = collections.deque([float('nan')] * WINDOW, maxlen=WINDOW)
sign_chg_buf = collections.deque([float('nan')] * WINDOW, maxlen=WINDOW)

paused     = [False]
poll_times = collections.deque(maxlen=500)

# Matches: [rpm] speed=1.23mph  encVel=0.012mph  cogging=0  sc=7
PATTERN = re.compile(r'\[rpm\] speed=([-\d.]+)mph\s+encVel=([-\d.]+)mph\s+cogging=(\d)\s+sc=(\d+)')

def serial_reader():
    with serial.Serial(port, BAUD, timeout=1) as ser:
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
            m = PATTERN.search(line)
            if not m or paused[0]:
                continue
            poll_times.append(time.time())
            speed_buf.append(float(m.group(1)))
            enc_vel_buf.append(float(m.group(2)))
            cogging_buf.append(float(m.group(3)))
            sign_chg_buf.append(float(m.group(4)))

thread = threading.Thread(target=serial_reader, daemon=True)
thread.start()

fig, (ax_speed, ax_cogging) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
fig.subplots_adjust(bottom=0.12, hspace=0.35)
fig.suptitle('RPM — Speed & Cogging')

# Speed panel — hall and encoder velocity overlaid
line_speed,   = ax_speed.plot([], [], color='tab:blue',   linewidth=1.2, label='hall speed')
line_enc_vel, = ax_speed.plot([], [], color='tab:purple', linewidth=1.0, label='enc velocity', alpha=0.8)
ax_speed.set_xlim(0, WINDOW)
ax_speed.set_ylim(-1.5, 3.0)
ax_speed.axhline(0, color='gray', linewidth=0.5, linestyle='--')
ax_speed.set_ylabel('speed (mph)')
ax_speed.legend(loc='upper right', fontsize=8)

# Cogging panel — binary state (left axis) + sign changes (right axis)
line_cogging, = ax_cogging.plot([], [], color='tab:red', linewidth=1.2, label='cogging')
ax_cogging.set_xlim(0, WINDOW)
ax_cogging.set_ylim(-0.1, 1.1)
ax_cogging.set_yticks([0, 1])
ax_cogging.set_yticklabels(['clear', 'cog'])
ax_cogging.set_ylabel('cogging', color='tab:red')
ax_cogging.tick_params(axis='y', labelcolor='tab:red')
ax_cogging.set_xlabel(f'samples  (window = {WINDOW} samples @ 200 Hz ≈ {WINDOW // 200}s)')

ax_cogging_r = ax_cogging.twinx()
line_sign_chg, = ax_cogging_r.plot([], [], color='tab:orange', linewidth=1.0, alpha=0.8, label='sign changes')
ax_cogging_r.set_ylim(0, RPM_COGGING_WINDOW_MAX)
ax_cogging_r.set_ylabel('sign changes', color='tab:orange')
ax_cogging_r.tick_params(axis='y', labelcolor='tab:orange')

lines_cog  = [line_cogging, line_sign_chg]
labels_cog = [l.get_label() for l in lines_cog]
ax_cogging.legend(lines_cog, labels_cog, loc='upper right', fontsize=8)

xs = list(range(WINDOW))

# Pause button
ax_button = fig.add_axes([0.82, 0.02, 0.12, 0.05])
btn_pause = Button(ax_button, 'Pause')

def on_pause(_):
    paused[0] = not paused[0]
    btn_pause.label.set_text('Resume' if paused[0] else 'Pause')
btn_pause.on_clicked(on_pause)

def poll_rate_hz():
    now = time.time()
    return float(sum(1 for t in poll_times if now - t <= 1.0))

def update(_):
    if paused[0]:
        return line_speed, line_enc_vel, line_cogging, line_sign_chg

    sdata = list(speed_buf)
    edata = list(enc_vel_buf)
    cdata = list(cogging_buf)
    scdata = list(sign_chg_buf)

    line_speed.set_data(xs, sdata)
    line_enc_vel.set_data(xs, edata)
    line_cogging.set_data(xs, cdata)
    line_sign_chg.set_data(xs, scdata)

    hz      = poll_rate_hz()
    valid_s  = [v for v in sdata  if v == v]
    valid_e  = [v for v in edata  if v == v]
    valid_c  = [v for v in cdata  if v == v]
    valid_sc = [v for v in scdata if v == v]

    speed_now   = valid_s[-1]      if valid_s  else 0.0
    enc_now     = valid_e[-1]      if valid_e  else 0.0
    cogging_now = int(valid_c[-1]) if valid_c  else 0
    sc_now      = int(valid_sc[-1]) if valid_sc else 0

    ax_speed.set_title(
        f'hall={speed_now:.3f} mph  enc={enc_now:.3f} mph  |  poll={hz:.0f} Hz', fontsize=9)
    ax_cogging.set_title(
        f'cogging={"YES" if cogging_now else "no"}  sc={sc_now}', fontsize=9)

    return line_speed, line_enc_vel, line_cogging, line_sign_chg

ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
plt.tight_layout()
plt.show()
