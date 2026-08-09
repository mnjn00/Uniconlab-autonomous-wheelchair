#!/usr/bin/env python3
"""Verdict on a DWA run: did the S-curve go away?

Reads a blackbox bag and prints the four numbers that defined the 2026-08-08
failure, against that run as the baseline. The point is that "it looked
better" is not an answer - the failure was specific and so is the test.

    rosrun static_livox_localization dwa_run_report.py <bag>
    python3 tools/dwa_run_report.py ~/localization_trials/blackbox_*.bag

What it measures, and why each one:

  yaw saturation      The planner asking for +-MAX_YAW_RATE. A position-only
                      cost is a bang-bang regulator and this is its
                      fingerprint. Was 50 % and 49 %.
  target_w reversals  How often the planner changed its mind about which way
                      to turn, as a rate. Was one every 1.8 s. Compare the
                      rate, not the count - a longer run has more of them.
  heading error       What the reversals were actually oscillating about:
                      cross-track was only 0.19 m at the reversals while
                      heading error was 20 deg.
  stop time           Standing still used to outscore every moving arc, for
                      180 s in one continuous block.

A pass looks like: saturation in low single digits, reversal rate well under
one per 2 s, and route covered that is not 12 % of the line.
"""

import collections
import math
import re
import sys

BASELINE = {
    "20260808_193013": dict(covered="8-44 m of 380", sat=50, rev_s=1.8,
                            herr=6, hold=181, label="2026-08-08 run 1"),
    "20260808_195800": dict(covered="8-48 m of 380", sat=49, rev_s=1.8,
                            herr=5, hold=79, label="2026-08-08 run 2"),
}

DWA = re.compile(r"DWA wp=(\d+)/(\d+) v=([-\d.]+) w=([-+\d.]+) "
                 r"target ([-\d.]+)/([-+\d.]+)")


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read(path):
    import rosbag
    status, pose, cmd = [], [], []
    with rosbag.Bag(path) as bag:
        t0 = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(
                topics=["/waypoint_follower/status", "/fast_lio_icp/pose",
                        "/cmd_vel_raw"]):
            t = stamp.to_sec() - t0
            if topic == "/waypoint_follower/status":
                status.append((t, msg.data))
            elif topic == "/cmd_vel_raw":
                cmd.append((t, msg.linear.x, msg.angular.z))
            else:
                p = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
                pose.append((t, p.position.x, p.position.y,
                             yaw_of(p.orientation)))
    return status, pose, cmd


def spans(times, gap=0.5):
    """Contiguous runs of samples, as (start, end) - a hold that is one block
    of 180 s and one that is 180 one-second blips are different faults."""
    out, start, last = [], None, None
    for t in times:
        if start is None:
            start = t
        elif t - last > gap:
            out.append((start, last))
            start = t
        last = t
    if start is not None:
        out.append((start, last))
    return out


def report(path):
    import numpy as np
    status, pose, cmd = read(path)
    if not status:
        print("  %s: no /waypoint_follower/status in this bag" % path)
        return
    duration = max(t for t, _ in status)
    key = next((k for k in BASELINE if k in path), None)
    base = BASELINE.get(key)

    holds = collections.Counter(
        d.split(":", 1)[1] for _t, d in status if d.startswith("HOLD:"))
    rows = [(t,) + tuple(m.groups())
            for t, d in status for m in [DWA.search(d)] if m]

    print("=" * 72)
    print("  %s" % path.rsplit("/", 1)[-1])
    print("  길이 %.0f s,  DWA 명령 표본 %d개 (%.0f s 상당)"
          % (duration, len(rows), len(rows) / 10.0))
    if not rows:
        print("  DWA 명령이 하나도 없음 - 프로파일이 dwa 가 아니었거나 "
              "한 번도 주행 상태에 들어가지 못함")
        print("  HOLD: " + ", ".join("%s x%d" % kv for kv in holds.most_common(6)))
        return

    wp = np.array([int(r[1]) for r in rows])
    total = int(rows[0][2])
    tw = np.array([float(r[6]) for r in rows])
    tv = np.array([float(r[5]) for r in rows])
    ts = np.array([r[0] for r in rows])

    sat = 100.0 * np.mean(np.abs(np.abs(tw) - 0.5) < 1e-6)
    sign = np.sign(tw)
    sign = sign[sign != 0]
    rev = int(np.sum(np.diff(sign) != 0)) if len(sign) > 1 else 0
    flips = np.where(np.diff(np.sign(tw)) != 0)[0]
    period = float(np.median(np.diff(ts[flips]))) if len(flips) > 2 else float("nan")

    print("  경로 진행     : wp %d -> %d / %d  (%.0f %%)"
          % (wp.min(), wp.max(), total, 100.0 * wp.max() / total))
    print("  요레이트 포화 : %.0f %%%s" % (sat, "   [8/8: %d %%]" % base["sat"] if base else ""))
    print("  목표w 부호반전: %d회,  중앙 간격 %.1f s%s"
          % (rev, period, "   [8/8: 1.8 s]" if base else ""))
    print("  목표 v        : 중앙 %.2f,  최대 %.2f" % (np.median(tv), tv.max()))

    if pose:
        P = np.array(pose)
        track = float(np.sum(np.linalg.norm(np.diff(P[:, 1:3], axis=0), axis=1)))
        net = float(np.linalg.norm(P[-1, 1:3] - P[0, 1:3]))
        print("  궤적          : 길이 %.0f m,  순변위 %.0f m" % (track, net))

    hold_s = {}
    for name in holds:
        times = [t for t, d in status if d == "HOLD:" + name]
        blocks = spans(times)
        hold_s[name] = (sum(b - a for a, b in blocks), max(
            (b - a for a, b in blocks), default=0.0), len(blocks))
    print("  HOLD 내역     :")
    for name, (tot, longest, n) in sorted(hold_s.items(), key=lambda x: -x[1][0])[:6]:
        print("     %-28s 합 %5.0f s,  최장 %5.0f s,  %d구간" % (name, tot, longest, n))
    if base:
        print("     [8/8 %s: DWA_BLOCKED 합 %d s]" % (base["label"], base["hold"]))

    print()
    verdict = []
    verdict.append(("요레이트 포화이 한 자릿수", sat < 10.0))
    verdict.append(("부호반전 간격 2 s 초과", not (period < 2.0)))
    verdict.append(("정지 교착 60 s 미만",
                    max((v[1] for k, v in hold_s.items() if "DWA_" in k),
                        default=0.0) < 60.0))
    verdict.append(("경로 30 % 이상 진행", 100.0 * wp.max() / total >= 30.0))
    for text, ok in verdict:
        print("  [%s] %s" % ("O" if ok else "X", text))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for arg in sys.argv[1:]:
        report(arg)
