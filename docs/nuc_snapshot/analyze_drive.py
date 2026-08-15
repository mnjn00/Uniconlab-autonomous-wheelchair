#!/usr/bin/env python3
"""오늘 DWA 주행의 블랙박스를 요약한다."""
import sys
from collections import Counter, OrderedDict

import rosbag

BAG = sys.argv[1]
bag = rosbag.Bag(BAG)

states = Counter()
first_t = last_t = None
wp_series = []       # (t, wp)
speed_series = []    # (t, cmd_vel.linear.x)
hold_spans = []      # (state, start_t)
current_hold = None

for topic, msg, t in bag.read_messages(
        topics=["/waypoint_follower/status", "/cmd_vel"]):
    ts = t.to_sec()
    if first_t is None:
        first_t = ts
    last_t = ts
    if topic == "/waypoint_follower/status":
        text = msg.data
        head = text.split()[0] if text.split() else text
        states[head] += 1
        if "wp=" in text:
            try:
                wp = int(text.split("wp=")[1].split("/")[0])
                wp_series.append((ts, wp))
            except (IndexError, ValueError):
                pass
        if head.startswith("HOLD"):
            if current_hold is None or current_hold[0] != head:
                if current_hold:
                    hold_spans.append((current_hold[0], current_hold[1], ts))
                current_hold = (head, ts)
        else:
            if current_hold:
                hold_spans.append((current_hold[0], current_hold[1], ts))
                current_hold = None
    else:
        speed_series.append((ts, msg.linear.x))

if current_hold:
    hold_spans.append((current_hold[0], current_hold[1], last_t))
bag.close()

dur = (last_t - first_t) if first_t else 0
print("기록 %.0f초" % dur)
print()
print("=== follower 상태 분포 ===")
total = sum(states.values()) or 1
for name, n in states.most_common(10):
    print("  %-28s %6d  (%4.1f%%)" % (name, n, 100.0 * n / total))

if wp_series:
    print()
    print("=== 경로 진행 ===")
    print("  wp %d -> %d" % (wp_series[0][1], max(w for _, w in wp_series)))
    print("  구간 시간 %.0f초" % (wp_series[-1][0] - wp_series[0][0]))
    span = max(w for _, w in wp_series) - wp_series[0][1]
    print("  전진 %d 스테이션 ≈ %.0f m (0.2 m 간격)" % (span, span * 0.2))

moving = [v for _, v in speed_series if v > 0.02]
if speed_series:
    print()
    print("=== 명령 속도 (/cmd_vel) ===")
    print("  샘플 %d, 움직인 샘플 %d (%.0f%%)"
          % (len(speed_series), len(moving), 100.0 * len(moving) / len(speed_series)))
    if moving:
        print("  평균 %.2f m/s, 최대 %.2f m/s"
              % (sum(moving) / len(moving), max(moving)))

print()
print("=== 홀드 구간 (2초 이상) ===")
long_holds = [(s, a, b) for s, a, b in hold_spans if b - a >= 2.0]
agg = OrderedDict()
for s, a, b in long_holds:
    agg.setdefault(s, []).append(b - a)
for s, durs in sorted(agg.items(), key=lambda kv: -sum(kv[1])):
    print("  %-28s %2d회  총 %5.0f초  최장 %4.0f초"
          % (s, len(durs), sum(durs), max(durs)))
