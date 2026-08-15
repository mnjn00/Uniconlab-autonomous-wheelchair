#!/usr/bin/env python3
"""Live ROS check for the bridge against the REAL field topics.

Publishes stand-ins for the driver/localization topics, runs nothing that can
reach a motor (uart.py is deliberately NOT started), drives the bridge over its
loopback debug port, and watches mode_cmd for the e-stop result.
"""
import json
import socket
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Int16MultiArray, String

AUTO, MANUAL = 65, 77
mode_state = [AUTO]
mode_cmd_seen = []

failures = []


def check(name, cond, note=""):
    print("  %-58s %s%s" % (name, "PASS" if cond else "FAIL", "" if not note else "  (%s)" % note))
    if not cond:
        failures.append(name)


def on_mode_cmd(msg):
    mode_cmd_seen.append(int(msg.data))
    mode_state[0] = int(msg.data)          # emulate the motor controller echo
    rospy.loginfo("fixture: mode_cmd=%d -> wheel_status will echo it", msg.data)


def main():
    rospy.init_node("bridge_live_test", anonymous=True, disable_signals=True)
    ws = rospy.Publisher("/wheel_status", Int16MultiArray, queue_size=1)
    od = rospy.Publisher("/Odometry", Odometry, queue_size=1)
    raw = rospy.Publisher("/cmd_vel_raw", Twist, queue_size=1)
    gated = rospy.Publisher("/cmd_vel_gated", Twist, queue_size=1)
    tip = rospy.Publisher("/tip_guard/status", String, queue_size=1)
    objs = rospy.Publisher("/perception/objects_summary", String, queue_size=1)
    fstat = rospy.Publisher("/waypoint_follower/status", String, queue_size=1)
    rospy.Subscriber("/mode_cmd", Int16, on_mode_cmd, queue_size=5)

    speed = [0.0]
    blocked = [False]

    def pump():
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            m = Int16MultiArray()
            # 72='H' header, data[1]=mode echo, data[7]=battery
            m.data = [72, mode_state[0], 0, 0, 0, 0, 0, 87]
            ws.publish(m)
            o = Odometry()
            o.twist.twist.linear.x = speed[0]
            od.publish(o)
            t = Twist()
            t.linear.x = 0.4 if blocked[0] else speed[0]
            raw.publish(t)
            g = Twist()
            g.linear.x = 0.0 if blocked[0] else speed[0]
            gated.publish(g)
            tip.publish(String(data="OK"))
            objs.publish(String(data="clusters=2 nearest=3.1m"))
            fstat.publish(String(data="RUN wp=4/31 v=0.40"))
            r.sleep()

    threading.Thread(target=pump, daemon=True).start()
    time.sleep(2.0)

    cli = socket.create_connection(("127.0.0.1", 8765))
    stream = cli.makefile("rb")

    def nxt(kind, timeout=8.0):
        end = time.time() + timeout
        while time.time() < end:
            line = stream.readline()
            if not line:
                return None
            obj = json.loads(line.decode())
            if obj.get("type") == kind:
                return obj
        return None

    def send(o):
        cli.sendall((json.dumps(o) + "\n").encode())

    def fresh_telemetry(n=4):
        f = None
        for _ in range(n):
            f = nxt("telemetry")
        return f

    print("\n--- live ROS check (real field topics) ---")
    f = fresh_telemetry()
    check("bridge sees ROS", f["ros_connected"] is True)
    check("drive_mode read from /wheel_status", f["drive_mode"] == "auto", f["drive_mode"])
    check("wheel link reported healthy", f["wheel_link_ok"] is True)

    speed[0] = 0.42
    f = fresh_telemetry()
    check("speed read from /Odometry", abs((f["speed_mps"] or 0) - 0.42) < 0.02,
          "%s m/s" % f["speed_mps"])
    check("battery read from /wheel_status data[7]", f["battery_percent"] == 87,
          str(f["battery_percent"]))
    check("follower status relayed", (f["follower_status"] or "").startswith("RUN"),
          f["follower_status"])
    check("objects_summary relayed", f["objects_summary"] is not None)
    check("ready_to_drive false while localization is not TRACKING",
          f["ready_to_drive"] is False, "localization=%s" % f["localization_status"])

    blocked[0] = True
    f = fresh_telemetry()
    check("safety_gate hold inferred (raw>0, gated==0)", f["motion_blocked"] is True)
    blocked[0] = False
    speed[0] = 0.0
    f = fresh_telemetry()
    check("hold clears when gate reopens", f["motion_blocked"] is False)

    print("\n  -- E-STOP --")
    mode_cmd_seen.clear()
    send({"command": "estop"})
    ack = nxt("ack")
    check("estop acked ok", ack is not None and ack["ok"] is True)
    time.sleep(1.0)
    check("mode_cmd=77 actually published", 77 in mode_cmd_seen, str(mode_cmd_seen))
    f = fresh_telemetry(6)
    check("telemetry confirms estop_engaged via wheel echo", f["estop_engaged"] is True)
    check("display_safe_to_drive false under estop", f["display_safe_to_drive"] is False)

    print("\n  -- release guards --")
    send({"command": "estop_release"})
    ack = nxt("ack")
    check("release without confirm refused", ack is not None and ack["ok"] is False)

    speed[0] = 0.30
    time.sleep(0.6)
    send({"command": "estop_release", "confirm": True})
    ack = nxt("ack")
    check("release refused while still moving",
          ack is not None and ack["ok"] is False and "moving" in ack["detail"],
          (ack or {}).get("detail", "")[:60])

    speed[0] = 0.0
    time.sleep(0.8)
    mode_cmd_seen.clear()
    send({"command": "estop_release", "confirm": True})
    ack = nxt("ack")
    check("release accepted when stopped + confirmed", ack is not None and ack["ok"] is True)
    time.sleep(1.0)
    check("mode_cmd=65 actually published", 65 in mode_cmd_seen, str(mode_cmd_seen))
    f = fresh_telemetry(6)
    check("telemetry shows released", f["estop_engaged"] is False)

    print("\n  -- drive start guards --")
    send({"command": "drive_start"})
    ack = nxt("ack")
    check("drive_start without confirm refused", ack is not None and ack["ok"] is False)

    send({"command": "estop"})
    nxt("ack")
    time.sleep(1.2)
    fresh_telemetry(6)
    send({"command": "drive_start", "confirm": True})
    ack = nxt("ack")
    check("drive_start refused while E-STOP engaged",
          ack is not None and ack["ok"] is False and "E-STOP" in ack["detail"],
          (ack or {}).get("detail", "")[:60])

    # release, then confirm go.sh's localization precondition is mirrored
    speed[0] = 0.0
    time.sleep(0.8)
    send({"command": "estop_release", "confirm": True})
    nxt("ack")
    time.sleep(1.2)
    fresh_telemetry(6)
    send({"command": "drive_start", "confirm": True})
    ack = nxt("ack")
    check("drive_start refused when localization is not TRACKING",
          ack is not None and ack["ok"] is False and "TRACKING" in ack["detail"],
          (ack or {}).get("detail", "")[:70])

    # leave the fixture in the released state
    speed[0] = 0.0
    time.sleep(0.8)
    send({"command": "estop_release", "confirm": True})
    nxt("ack")

    cli.close()
    print("\n--- %s ---\n" % ("all checks passed" if not failures
                              else "%d FAILED: %s" % (len(failures), ", ".join(failures))))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
