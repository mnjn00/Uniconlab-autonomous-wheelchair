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

The route IS the resampled trace. It is not a set of chosen waypoints, because
the band is computed about the route - it casts its lateral rays perpendicular
to it - so any departure from the driven path becomes a clearance measured off
a line the chair was never on, at an angle it was never at.

Two spacing rules were tried and both put that error back by construction. A
curvature estimate left 5-6 m gaps spanning up to 131 degrees of real turning,
with the driven path 1.36 m from its own recorded line and 10 percent of band
stations refusing the line the chair demonstrably drove. Bounding the chord
sag brought that to 0.39 m, which is better and still an invented displacement
in a budget whose margin is 0.10 m. Carrying every resampled point removes the
question: the only departure left is the resampling step itself.

The trace is smoothed nowhere. Every consumer already averages over at least a
metre - the band takes its station heading over a 2 m baseline, pure pursuit
interpolates a lookahead 0.9 m or further ahead - and a moving average was
measured displacing the line by up to 0.55 m at corners, which is the error
being removed.

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

POLYLINE_STEP = 0.2
# The start heading is compared against the direction the route first travels,
# and over one 0.2 m step that direction is noise: the raw polyline turns by up
# to 27 deg between consecutive steps. Measure the travel direction over at
# least this much route instead.
GUARD_BASELINE_M = 1.0


def polyline(xy, z, yaw):
    """Resample to a fixed step, carrying the recorded height and heading."""
    points, heights, headings = [xy[0]], [z[0]], [yaw[0]]
    for p, pz, py in zip(xy, z, yaw):
        if np.linalg.norm(p - points[-1]) >= POLYLINE_STEP:
            points.append(p)
            heights.append(pz)
            headings.append(py)
    # The final partial step is still route. Dropping it ended the route up to
    # POLYLINE_STEP short of where the drive did, which is the opposite of
    # keeping the recording end to end.
    if len(xy) > 1 and not np.array_equal(points[-1], xy[-1]):
        points.append(xy[-1])
        heights.append(z[-1])
        headings.append(yaw[-1])
    return np.array(points), np.array(heights), np.array(headings)






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

    waypoints = [{
        "x": round(float(line[i][0]), 3),
        "y": round(float(line[i][1]), 3),
        "z": round(float(heights[min(i, len(heights) - 1)]), 2),
        "yaw_deg": round(float(np.degrees(
            headings[min(i, len(headings) - 1)])), 1),
    } for i in range(len(line))]

    # The first waypoint is the auto-init seed, so a backwards heading
    # there costs a whole initialization. Compare it against the direction
    # the route immediately travels: a chair cannot be driving forwards
    # while facing away from where it goes.
    ahead = next(
        (w for w in waypoints
         if math.hypot(w["x"] - waypoints[0]["x"],
                       w["y"] - waypoints[0]["y"]) >= GUARD_BASELINE_M),
        waypoints[-1])
    travel = math.degrees(math.atan2(
        ahead["y"] - waypoints[0]["y"],
        ahead["x"] - waypoints[0]["x"]))
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
