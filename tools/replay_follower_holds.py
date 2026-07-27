#!/usr/bin/env python3
"""Replay a debug bag through waypoint_follower's hold logic.

The bag answers "were the sensors healthy". This answers the next question:
given those exact recorded conditions, would the follower have been allowed
to drive? Every hold branch in WaypointFollower.step() is evaluated against
the recorded pose, tilt, wheel status and localization state, in the branch
order the follower itself uses, so the reported reason is the one that would
actually have won.

Offline only: reads with `rosbags`, touches no ROS and no NUC.
"""

import argparse
import json
import math
import sys
from collections import Counter

import numpy as np
from rosbags.highlevel import AnyReader
from pathlib import Path

# Constants read from the rolled-back follower (999028a).
MAX_TILT_ROLL = math.radians(6.0)
MAX_TILT_PITCH = math.radians(8.0)
GEOFENCE_M = 3.5
BAND_RECOVER_MAX = 0.5
CHAIR_HALF_WIDTH = 0.35
BAND_MARGIN = 0.10
BAND_FLOOR = 0.15
NARROW_BAND_WIDTH = 1.2
AUTO_MODE = 65
BASE_STALE_S = 1.5
POSE_STALE_S = 1.0


class Band:
    """The rolled-back SafetyBand: limits are the MORE PERMISSIVE of the two
    nearest stations (max), which is the behaviour on the chair today."""

    def __init__(self, path):
        data = json.load(open(path))
        self.xy = np.array([[s["x"], s["y"]] for s in data["stations"]])
        head = np.radians([s["heading_deg"] for s in data["stations"]])
        self.normals = np.stack([-np.sin(head), np.cos(head)], axis=1)
        self.left = np.array([max(s["left_m"] - CHAIR_HALF_WIDTH - BAND_MARGIN,
                                  BAND_FLOOR) for s in data["stations"]])
        self.right = np.array([max(s["right_m"] - CHAIR_HALF_WIDTH - BAND_MARGIN,
                                   BAND_FLOOR) for s in data["stations"]])
        self.narrow = np.array([s["left_m"] + s["right_m"] < NARROW_BAND_WIDTH
                                for s in data["stations"]])

    def contains(self, point, grace=0.0):
        d = np.linalg.norm(self.xy - point, axis=1)
        order = np.argsort(d)[:2]
        k = int(order[0])
        lateral = float(np.dot(point - self.xy[k], self.normals[k]))
        lo = -max(self.right[j] for j in order)
        hi = max(self.left[j] for j in order)
        return lo - grace - 1e-6 <= lateral <= hi + grace + 1e-6


def rpy(q):
    x, y, z, w = q
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return roll, pitch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--route", required=True)
    ap.add_argument("--band", required=True)
    args = ap.parse_args()

    route = np.array([[w["x"], w["y"]]
                      for w in json.load(open(args.route))["waypoints"]])
    band = Band(args.band)

    poses, modes, wheel_t, diag = [], [], [], []
    with AnyReader([Path(args.bag)]) as reader:
        for conn, t, raw in reader.messages():
            ts = t * 1e-9
            if conn.topic == "/fast_lio_icp/pose":
                m = reader.deserialize(raw, conn.msgtype)
                p = m.pose.pose.position
                o = m.pose.pose.orientation
                poses.append((ts, p.x, p.y, o.x, o.y, o.z, o.w))
            elif conn.topic == "/wheel_status":
                m = reader.deserialize(raw, conn.msgtype)
                wheel_t.append(ts)
                if len(m.data) > 1:
                    modes.append(int(m.data[1]))
            elif conn.topic == "/fast_lio_icp/localization_diagnostics":
                m = reader.deserialize(raw, conn.msgtype)
                st = None
                for s in m.status:
                    for kv in s.values:
                        if kv.key in ("raw_state", "tracking_state"):
                            st = kv.value
                if st:
                    diag.append((ts, st))

    if not poses:
        print("no /fast_lio_icp/pose in bag")
        return 1
    P = np.array(poses)
    t0 = P[0, 0]
    print("poses: %d over %.1f s" % (len(P), P[-1, 0] - P[0, 0]))

    # --- tilt ---
    rolls, pitches = [], []
    for row in P:
        r, p = rpy(row[3:7])
        rolls.append(r)
        pitches.append(p)
    rolls = np.degrees(np.array(rolls))
    pitches = np.degrees(np.array(pitches))
    tilt_hold = (np.abs(rolls) > math.degrees(MAX_TILT_ROLL)) | \
                (np.abs(pitches) > math.degrees(MAX_TILT_PITCH))
    print("\n--- TILT_LIMIT (limits: roll %.0f deg, pitch %.0f deg) ---"
          % (math.degrees(MAX_TILT_ROLL), math.degrees(MAX_TILT_PITCH)))
    print("roll  range %+.2f .. %+.2f  |  pitch range %+.2f .. %+.2f"
          % (rolls.min(), rolls.max(), pitches.min(), pitches.max()))
    print("roll  over limit: %5.1f%% of samples" %
          (100.0 * np.mean(np.abs(rolls) > math.degrees(MAX_TILT_ROLL))))
    print("pitch over limit: %5.1f%% of samples" %
          (100.0 * np.mean(np.abs(pitches) > math.degrees(MAX_TILT_PITCH))))
    print("TILT_LIMIT would hold: %5.1f%% of the drive" %
          (100.0 * np.mean(tilt_hold)))

    # --- route containment ---
    d_route = np.array([np.min(np.linalg.norm(route - P[i, 1:3], axis=1))
                        for i in range(len(P))])
    off_route = d_route > GEOFENCE_M
    in_band = np.array([band.contains(P[i, 1:3], grace=BAND_RECOVER_MAX)
                        for i in range(len(P))])
    print("\n--- OFF_ROUTE / OFF_BAND ---")
    print("distance to nearest waypoint: median %.2f m  max %.2f m"
          % (np.median(d_route), d_route.max()))
    print("OFF_ROUTE would hold: %5.1f%% (geofence %.1f m)"
          % (100.0 * np.mean(off_route), GEOFENCE_M))
    print("OFF_BAND  would hold: %5.1f%% (grace %.2f m)"
          % (100.0 * np.mean(~in_band), BAND_RECOVER_MAX))

    # --- drive mode ---
    print("\n--- MANUAL_MODE ---")
    if modes:
        c = Counter(modes)
        print("wheel_status mode bytes: %s" % dict(c.most_common(5)))
        print("fraction reporting AUTO(65): %.1f%%"
              % (100.0 * sum(v for k, v in c.items() if k == AUTO_MODE)
                 / sum(c.values())))
    else:
        print("no decodable mode byte in /wheel_status")

    # --- base staleness ---
    if len(wheel_t) > 1:
        gaps = np.diff(np.array(wheel_t))
        print("\n--- BASE_STALE (limit %.1f s) ---" % BASE_STALE_S)
        print("wheel_status gap: median %.3f s  max %.3f s  over limit: %d"
              % (np.median(gaps), gaps.max(), int(np.sum(gaps > BASE_STALE_S))))

    # --- localization ---
    if diag:
        c = Counter(s for _, s in diag)
        print("\n--- LOCALIZATION_LOST ---")
        print("states: %s" % dict(c))

    # --- combined, in the follower's own branch order ---
    print("\n--- winning hold reason, evaluated per pose sample ---")
    reasons = Counter()
    for i in range(len(P)):
        if tilt_hold[i]:
            reasons["TILT_LIMIT"] += 1
        elif off_route[i]:
            reasons["OFF_ROUTE"] += 1
        elif not in_band[i]:
            reasons["OFF_BAND"] += 1
        else:
            reasons["(would drive)"] += 1
    total = len(P)
    for k, v in reasons.most_common():
        print("  %-14s %6.1f%%  (%d samples)" % (k, 100.0 * v / total, v))
    return 0


if __name__ == "__main__":
    sys.exit(main())
