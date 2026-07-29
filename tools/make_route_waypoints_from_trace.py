#!/usr/bin/env python3
"""Curvature-adaptive waypoints from a FULL localization pose trace.

Unlike make_route_waypoints.py, which keeps only the outbound
start->farthest leg of a mapping drive, this keeps the whole recording:
first metre to last. Written after a waypoint export trimmed real
driving distance off both ends of the 0727 route along with its
spin-in-place bookends. No trimming is needed - a spin contributes no
displacement, so the 0.2 m polyline step absorbs it on its own, and the
chair can then start from where it was actually parked instead of
5.3 m down the path (past the follower's 3.5 m geofence).

Straights get waypoints up to 6 m apart, curves down to 1.5 m.

The route records the body-frame profile it was captured under: FAST-LIO
reports the pose of its IMU body frame, so a route driven on a different
inertial profile than it was recorded on measures the chair against the
wrong origin. The follower refuses a route whose profile does not match
its own.

Each waypoint's yaw is the heading the localizer RECORDED there, not one
derived from the polyline. Deriving it looked equivalent and was not: at
the start the chair is turning on the spot, so the direction between
consecutive polyline points is arbitrary, and it came out 180 deg
backwards. That first waypoint is what auto-initialization seeds as its
known-start prior, so the seed told the localizer the chair faced the
wrong way and verification could never converge - the failure that sent
initialization into its global fallback every time.

Input trace: whitespace columns "t x y z yaw_rad" in the map frame,
exported from /fast_lio_icp/pose. A four-column trace without yaw is
rejected rather than silently falling back to the derived heading.

Usage:
  make_route_waypoints_from_trace.py <trace.txt> <profile> <source> <out.json>
"""

import json
import math
import sys

import numpy as np

MIN_SPACING = 1.5
MAX_SPACING = 6.0
POLYLINE_STEP = 0.2
CURVE_FULL_DENSITY = 0.35  # rad/m at which spacing reaches MIN_SPACING


def polyline(xy, z, yaw):
    """Resample to a fixed step, carrying the recorded height and heading."""
    points, heights, headings = [xy[0]], [z[0]], [yaw[0]]
    for p, pz, py in zip(xy, z, yaw):
        if np.linalg.norm(p - points[-1]) >= POLYLINE_STEP:
            points.append(p)
            heights.append(pz)
            headings.append(py)
    return np.array(points), np.array(heights), np.array(headings)


def smooth(points, kernel=5):
    padded = np.vstack([points[:1].repeat(kernel, 0), points,
                        points[-1:].repeat(kernel, 0)])
    stack = np.vstack([
        padded[i:len(padded) - 2 * kernel + i] for i in range(2 * kernel + 1)
    ]).reshape(2 * kernel + 1, -1, 2)
    return stack.mean(axis=0)


def curvature(points, headings, window=5):
    values = np.zeros(len(points))
    for i in range(len(points)):
        a = headings[max(0, i - window)]
        b = headings[min(len(headings) - 1, i + window - 1)]
        turn = np.arctan2(np.sin(b - a), np.cos(b - a))
        arc = POLYLINE_STEP * (min(len(headings) - 1, i + window - 1)
                               - max(0, i - window) + 1)
        values[i] = abs(turn) / max(arc, POLYLINE_STEP)
    return values


def main():
    if len(sys.argv) != 5:
        raise SystemExit(
            "usage: make_route_waypoints_from_trace.py "
            "<trace.txt> <profile> <source> <out.json>")
    trace_path, profile, source, out_path = sys.argv[1:5]

    rows = np.loadtxt(trace_path)
    if rows.shape[1] < 5:
        raise SystemExit(
            "trace needs columns 't x y z yaw_rad'; a positions-only trace "
            "would force the heading to be derived, which is what put the "
            "start waypoint 180 deg backwards")
    line, heights, headings = polyline(rows[:, 1:3], rows[:, 3], rows[:, 4])
    length = np.linalg.norm(np.diff(line, axis=0), axis=1).sum()
    print("trace %d poses -> polyline %d pts, %.0f m (kept end to end)"
          % (len(rows), len(line), length))

    curve = smooth(line)
    # Curvature still comes from the smoothed geometry - that is what sets
    # waypoint spacing - but it is kept separate from the heading written
    # into each waypoint, which stays the recorded one.
    density = curvature(curve, np.arctan2(*np.diff(curve, axis=0).T[::-1]))

    picked, travelled = [0], 0.0
    for i in range(1, len(curve)):
        travelled += float(np.linalg.norm(curve[i] - curve[i - 1]))
        ratio = min(density[i] / CURVE_FULL_DENSITY, 1.0)
        if travelled >= MAX_SPACING - ratio * (MAX_SPACING - MIN_SPACING):
            picked.append(i)
            travelled = 0.0
    if picked[-1] != len(curve) - 1:
        picked.append(len(curve) - 1)

    waypoints = [{
        "x": round(float(curve[i][0]), 2),
        "y": round(float(curve[i][1]), 2),
        "z": round(float(heights[min(i, len(heights) - 1)]), 2),
        "yaw_deg": round(float(np.degrees(
            headings[min(i, len(headings) - 1)])), 1),
    } for i in picked]

    # The first waypoint is the auto-init seed, so a backwards heading
    # there costs a whole initialization. Compare it against the direction
    # the route immediately travels: a chair cannot be driving forwards
    # while facing away from where it goes.
    travel = math.degrees(math.atan2(
        waypoints[1]["y"] - waypoints[0]["y"],
        waypoints[1]["x"] - waypoints[0]["x"]))
    disagreement = abs((waypoints[0]["yaw_deg"] - travel + 180) % 360 - 180)
    print("start heading %.1f deg vs first travel direction %.1f deg "
          "(%.1f deg apart)"
          % (waypoints[0]["yaw_deg"], travel, disagreement))
    if disagreement > 90.0:
        raise SystemExit(
            "start waypoint faces %.0f deg away from the route it drives - "
            "refusing to write a route that would seed initialization "
            "backwards" % disagreement)

    with open(out_path, "w") as handle:
        json.dump({
            "frame": "map",
            "source": source,
            "body_frame_profile": profile,
            "count": len(waypoints),
            "waypoints": waypoints,
        }, handle, indent=1)
    print("wrote %d waypoints -> %s" % (len(waypoints), out_path))
    print("first %s" % waypoints[0])
    print("last  %s" % waypoints[-1])


if __name__ == "__main__":
    main()
