"""Put the VN-100 back into a state the ROS driver can open.

An unclean SIGTERM leaves the sensor streaming whatever the driver last
configured (binary, high baud). The next driver start reads that stream
while expecting ASCII register replies and segfaults, so async output is
switched off here first. Nothing is written to non-volatile memory: a
power cycle must still come up at the factory default.
"""
import time
import serial

PORT = "/dev/vn"
BAUDS = (921600, 115200, 230400, 460800, 57600, 38400, 19200, 9600)


def nmea(body):
    ck = 0
    for c in body:
        ck ^= ord(c)
    return ("$%s*%02X\r\n" % (body, ck)).encode()


def talk(s, body, wait=0.35):
    s.reset_input_buffer()
    s.write(nmea(body))
    time.sleep(wait)
    return s.read(600)


for baud in BAUDS:
    try:
        s = serial.Serial(PORT, baud, timeout=0.4)
    except Exception as e:
        print("%7d open failed: %s" % (baud, e))
        continue
    time.sleep(0.25)
    pending = s.read(3000)
    reply = talk(s, "VNRRG,01")
    ascii_ok = b"VNRRG,01" in reply
    print("%7d  streaming=%5d bytes  model_reply=%s"
          % (baud, len(pending),
             reply.decode("ascii", "replace").strip()[:60] if reply else "none"))
    if ascii_ok:
        off = talk(s, "VNWRG,06,0")          # async output type = none
        after = s.read(2000)
        print("  async-off ack: %s"
              % off.decode("ascii", "replace").strip()[:60])
        print("  bytes still arriving after disable: %d" % len(after))
        s.close()
        print("\nRESET_OK at %d" % baud)
        raise SystemExit(0)
    s.close()
print("\nRESET_FAILED - no baud produced an ASCII register reply")
raise SystemExit(1)
