#!/usr/bin/env python3
"""Read-only sniff of the wheel controller's UART reply frames.

Opens /dev/uart and never writes a byte, so it cannot command the base. Decodes
the frame the way odom_pub.py and uart.py do and reports what each field is
actually doing, which is the only way to settle what data[7] means.

usage: sniff_uart.py [seconds]
"""
import collections
import sys
import time

import serial

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0


def checksum(values):
    return (~(sum(values)) + 1) & 0xFF


def speed_kmh(direction, magnitude):
    if direction == 67:      # 'C'
        return (magnitude - 33) / 10.0
    if direction == 87:      # 'W'
        return -(magnitude - 33) / 10.0
    return 0.0               # 'S'


ser = serial.Serial(port="/dev/uart", baudrate=115200, bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                    timeout=0.05)

buf = []
frames = 0
bad = 0
seen = collections.defaultdict(int)
by_field = collections.defaultdict(lambda: collections.defaultdict(int))
first = None
last = None
start = time.time()
samples = []

while time.time() - start < SECONDS:
    chunk = ser.read(64)
    for byte in chunk:
        buf.append(byte)
        if buf[0] != 72:
            buf.pop(0)
            continue
        if len(buf) >= 3 and buf[-2:] == [13, 10]:
            if len(buf) < 6:
                buf = []
                continue
            payload = buf[1:-3]
            ok = buf[-3] == checksum(payload)
            if ok:
                frames += 1
                for i, v in enumerate(buf):
                    by_field[i][v] += 1
                seen[buf[7] if len(buf) > 7 else None] += 1
                if first is None:
                    first = list(buf)
                last = list(buf)
                if len(buf) > 7:
                    lv = speed_kmh(buf[2], buf[3])
                    rv = speed_kmh(buf[4], buf[5])
                    samples.append((time.time() - start, buf[6], buf[7],
                                    round((lv + rv) / 2.0, 2)))
            else:
                bad += 1
            buf = []
        if len(buf) > 40:
            buf = []

ser.close()
print("frames ok=%d bad=%d over %.0fs" % (frames, bad, SECONDS))
if not frames:
    print("NO FRAMES -- the wheel base is probably not powered.")
    raise SystemExit(1)
print("first frame: %s" % first)
print("last  frame: %s" % last)
print("frame length: %s" % sorted({len(f) for f in (first, last)}))
print()
for idx in sorted(by_field):
    values = by_field[idx]
    top = sorted(values.items(), key=lambda kv: -kv[1])[:6]
    label = {0: "header", 1: "mode echo", 2: "dir L", 3: "mag L",
             4: "dir R", 5: "mag R", 6: "?", 7: "? (reported as battery)"}.get(idx, "")
    print("data[%d] %-26s distinct=%-3d  %s"
          % (idx, label, len(values),
             "  ".join("%d x%d" % (v, c) for v, c in top)))
print()
print("time    data[6]  data[7]  wheel_speed_kmh")
step = max(1, len(samples) // 20)
for t, d6, d7, v in samples[::step]:
    print("%6.1f  %6d  %7d  %8.2f" % (t, d6, d7, v))
