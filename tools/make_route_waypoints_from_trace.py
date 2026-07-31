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

Spacing is set by how far the polyline is allowed to depart from the path it
replaces, not by a curvature estimate. The band is computed about the
polyline - it casts its lateral rays perpendicular to it - so wherever the
polyline cuts a corner, the band measures the clearance of a line the chair
was never on, at an angle the chair was never at. Bounding the sag bounds
exactly that error. Straights, where the sag is zero, are capped at 6 m.

Choosing spacing from curvature did not bound it: on the 0727 route it emitted
14 gaps of 5-6 m that each spanned 20 to 131 degrees of real turning, putting
the driven path up to 1.49 m from its own recorded line, and 10 percent of
band stations then refused the line the chair demonstrably drove.

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

MAX_SPACING = 6.0
POLYLINE_STEP = 0.2
# Largest perpendicular distance the polyline may sit from the path it
# replaces. This is an error in where the band believes the chair was, so it
# is sized against BAND_MARGIN (0.10 m) rather than against the 0.45 m
# clearance: at 0.10 m the polyline is never further from the truth than the
# margin the band is trying to hold.
MAX_SAG = 0.10
# Half-width of the moving average applied before waypoints are placed, in
# 0.2 m polyline steps. Smoothing is needed - the raw polyline turns by up to
# 27 deg between consecutive steps - but it displaces the line as well, and
# that displacement lands in the band exactly like a sampling error would.
# Measured on the 0727 trace: kernel 5 moves the line by up to 0.548 m,
# kernel 2 by 0.251 m, while still removing the step-to-step jitter.
SMOOTH_KERNEL = 2


def polyline(xy, z, yaw):
    """Resample to a fixed step, carrying the recorded height and heading."""
    points, heights, headings = [xy[0]], [z[0]], [yaw[0]]
    for p, pz, py in zip(xy, z, yaw):
        if np.linalg.norm(p - points[-1]) >= POLYLINE_STEP:
            points.append(p)
            heights.append(pz)
            headings.append(py)
    return np.array(points), np.array(heights), np.array(headings)


def smooth(points, kernel=SMOOTH_KERNEL):
    padded = np.vstack([points[:1].repeat(kernel, 0), points,
                        points[-1:].repeat(kernel, 0)])
    stack = np.vstack([
        padded[i:len(padded) - 2 * kernel + i] for i in range(2 * kernel + 1)
    ]).reshape(2 * kernel + 1, -1, 2)
    return stack.mean(axis=0)


def chord_sag(points, a, b):
    """Largest perpendicular distance from chord a->b to the arc between."""

    if b - a < 2:
        return 0.0
    start = points[a]
    span = points[b] - start
    length = float(np.hypot(span[0], span[1]))
    if length < 1e-9:
        return 0.0
    offsets = points[a + 1:b] - start
    cross = np.abs(offsets[:, 0] * span[1] - offsets[:, 1] * span[0]) / length
    return float(cross.max())


def pick_waypoints(points, max_sag=MAX_SAG, max_spacing=MAX_SPACING):
    """Indices whose polyline holds `points` to within `max_sag`.

    Greedy rather than optimal: extend the current span until either the sag
    or the chord length would break, then cut at the last index that held.
    Optimal placement would need a handful fewer waypoints and is not worth
    the complexity - the cost of an extra waypoint is nothing, and the cost
    of a span that lies about where the chair was is a band measured off the
    driven line.
    """

    count = len(points)
    if count <= 2:
        return list(range(count))
    picked = [0]
    anchor = 0
    for index in range(1, count):
        span = points[index] - points[anchor]
        too_long = float(np.hypot(span[0], span[1])) >= max_spacing
        if too_long or chord_sag(points, anchor, index) > max_sag:
            cut = index - 1 if index - 1 > anchor else index
            picked.append(cut)
            anchor = cut
    if picked[-1] != count - 1:
        picked.append(count - 1)
    return picked


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

    # Smoothing is for the geometry the waypoints sit on, not for the
    # decision: the sag is measured against the smoothed line so a waypoint
    # is not placed to chase sensor noise, while the heading written into
    # each waypoint stays the recorded one.
    curve = smooth(line)
    picked = pick_waypoints(curve)

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
