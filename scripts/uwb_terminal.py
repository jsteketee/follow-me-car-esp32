import serial, time, sys, threading

s = serial.Serial('/dev/tty.usbserial-0001', 115200, timeout=0.1)

def reader():
    while True:
        data = s.read(100)
        if data:
            print(data.decode(errors='replace'), end='', flush=True)

t = threading.Thread(target=reader, daemon=True)
t.start()

print("UWB terminal ready. Type commands and press Enter. Ctrl+C to quit.\n")
try:
    while True:
        cmd = input()
        s.write((cmd + '\r\n').encode())
except KeyboardInterrupt:
    s.close()
