import serial, time, threading, sys

freq = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0

s = serial.Serial('/dev/tty.usbserial-0001', 115200, timeout=1)

total_dist = 0.0
total_count = 0
start_time = time.time()
lock = threading.Lock()

def reader():
    global total_dist, total_count
    buf = ''
    while True:
        buf += s.read(100).decode(errors='replace')
        while '\n' in buf:
            line, buf = buf.split('\n', 1)
            if '+ANCHOR_RCV' in line:
                try:
                    dist_cm = int(line.split(',')[-1].strip().split()[0])
                except (ValueError, IndexError):
                    continue
                with lock:
                    total_count += 1
                    total_dist += dist_cm
                    elapsed = time.time() - start_time
                    polls_per_sec = total_count / elapsed if elapsed > 0 else 0
                    avg = total_dist / total_count
                print(f"Distance: {dist_cm}cm , Average: {avg:.0f}cm , Avg polls/sec: {polls_per_sec:.1f}")

threading.Thread(target=reader, daemon=True).start()

print(f"Polling at {freq} Hz... Ctrl+C to stop\n")
try:
    while True:
        s.write(b'AT+ANCHOR_SEND=TAG,4,TEST\r\n')
        time.sleep(1.0 / freq)
except KeyboardInterrupt:
    s.close()
