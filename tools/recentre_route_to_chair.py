#!/usr/bin/env python3
"""Re-express a recorded route about the chair centre instead of the sensor.

A route is a recording of FAST-LIO's body origin, and that origin is wherever
the lidar was bolted on - here the front of the LEFT armrest, 0.517 m forward
and 0.173 m left of the centre the chair turns about (measured; see
body_frame.CHAIR_CENTRE_IN_BODY_XYZ).

Nothing about the drive changes. The chair went where it went; this expresses
the same motion about the point every downstream constant already assumes it
is about. Once the route is chair-centred and the band is regenerated from it,
CHAIR_HALF_WIDTH either side is finally a true statement, instead of one that
over-protects the left by 0.173 m and under-protects the right by the same.

Heading is unchanged: moving the reference point along the body frame is a
translation, not a rotation.

Usage: recentre_route_to_chair.py <route.json> <out.json>
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "src", "static_livox_localization", "scripts"))

from body_frame import (  # noqa: E402
    CHAIR_CENTRE_IN_BODY_XYZ,
    REFERENCE_BODY,
    REFERENCE_CHAIR_CENTRE,
)


def recentre(route):
    """Move every waypoint from the sensor's path to the chair centre's."""

    current = route.get("reference_point", REFERENCE_BODY)
    if current != REFERENCE_BODY:
        raise SystemExit(
            "route is already about %s; re-centring it again would displace "
            "it a second time" % current)

    forward, left, _ = CHAIR_CENTRE_IN_BODY_XYZ
    moved = []
    for waypoint in route["waypoints"]:
        yaw = math.radians(waypoint["yaw_deg"])
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        shifted = dict(waypoint)
        shifted["x"] = round(
            waypoint["x"] + cos_yaw * forward - sin_yaw * left, 4)
        shifted["y"] = round(
            waypoint["y"] + sin_yaw * forward + cos_yaw * left, 4)
        moved.append(shifted)

    out = dict(route)
    out["waypoints"] = moved
    out["reference_point"] = REFERENCE_CHAIR_CENTRE
    out["recentred_from"] = route.get("source", "sensor-path route")
    out["chair_centre_in_body_xyz"] = list(CHAIR_CENTRE_IN_BODY_XYZ)
    return out


def main(route_path, out_path):
    route = json.load(open(route_path, encoding="utf-8"))
    out = recentre(route)

    shifts = [
        math.hypot(a["x"] - b["x"], a["y"] - b["y"])
        for a, b in zip(route["waypoints"], out["waypoints"])
    ]
    print("waypoints: %d" % len(shifts))
    print("shift: min %.3f  max %.3f  (expected %.3f m everywhere)" % (
        min(shifts), max(shifts),
        math.hypot(CHAIR_CENTRE_IN_BODY_XYZ[0], CHAIR_CENTRE_IN_BODY_XYZ[1])))
    print("start (%.3f, %.3f) -> (%.3f, %.3f)" % (
        route["waypoints"][0]["x"], route["waypoints"][0]["y"],
        out["waypoints"][0]["x"], out["waypoints"][0]["y"]))
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(out, handle, indent=1)
        handle.write("\n")
    print("wrote %s (reference_point=%s)" % (out_path, out["reference_point"]))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
