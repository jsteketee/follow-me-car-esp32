#!/usr/bin/env python3
# Live serial plotter for IMU linear acceleration (lax, lay, laz).
# Requires: pip install pyserial matplotlib
# Usage: python imu_plotter.py [port] [baud]
#   port defaults to auto-detect first /dev/cu.usbmodem* on macOS
#   baud defaults to 115200

import sys
import glob
import re
import collections
import threading
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PORT  = sys.argv[1] if len(sys.argv) > 1 else None
BAUD  = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
WINDOW = 300  # number of samples to display

def auto_detect_port():
    candidates = glob.glob('/dev/cu.usbmodem*') + glob.glob('/dev/cu.SLAB*') + glob.glob('/dev/cu.wchusbserial*')
    if candidates:
        return candidates[0]
    raise RuntimeError("No USB serial port found. Pass port as first argument.")

port = PORT or auto_detect_port()
print(f"Connecting to {port} at {BAUD} baud...")

buf = { 'lax': collections.deque([0.0]*WINDOW, maxlen=WINDOW),
        'lay': collections.deque([0.0]*WINDOW, maxlen=WINDOW),
        'laz': collections.deque([0.0]*WINDOW, maxlen=WINDOW) }

PATTERN = re.compile(r'lax:([-\d.]+),lay:([-\d.]+),laz:([-\d.]+)')

def serial_reader():
    with serial.Serial(port, BAUD, timeout=1) as ser:
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
            m = PATTERN.search(line)
            if m:
                buf['lax'].append(float(m.group(1)))
                buf['lay'].append(float(m.group(2)))
                buf['laz'].append(float(m.group(3)))

thread = threading.Thread(target=serial_reader, daemon=True)
thread.start()

fig, ax = plt.subplots()
fig.suptitle('IMU Linear Acceleration (m/s²) — gravity removed')
lines = {
    'lax': ax.plot([], [], label='lax (left/right)', color='tab:red')[0],
    'lay': ax.plot([], [], label='lay (fwd/back)',   color='tab:green')[0],
    'laz': ax.plot([], [], label='laz (up/down)',    color='tab:blue')[0],
}
ax.set_xlim(0, WINDOW)
ax.set_ylim(-15, 15)
ax.axhline(0, color='gray', linewidth=0.5)
ax.set_ylabel('m/s²')
ax.set_xlabel('samples')
ax.legend(loc='upper right')
xs = list(range(WINDOW))

def update(_):
    for key, line in lines.items():
        line.set_data(xs, list(buf[key]))
    return lines.values()

ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
plt.tight_layout()
plt.show()
