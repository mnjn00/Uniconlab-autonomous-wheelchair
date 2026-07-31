#!/usr/bin/env python3
"""Drivable corridor and no-go areas for the route, in map-frame polygons.

The safety band answers one question per station per side - how far can the
chair go before the ground breaks - and the follower reduces that to a lateral
clamp. Two things are lost. An area in the MIDDLE of the corridor cannot be
expressed at all, and the reason an edge is an edge is thrown away by the time
the follower sees it.

This turns the band back into geometry: a drivable ribbon, the non-drivable
strips outside every hard edge, and the places where the two leave too little
room to drive safely. Those last are what a person needs to look at, and the
output is deliberately hand-editable so zones the map cannot show - a glass
door, a vehicle crossing, a busy entrance - can be added to the same file.

The edge classification here is NOT the one the shipped band consumer uses:

    is_severe(drop_m) -> drop_m is None or drop_m < 0.0 or drop_m >= 0.12

That counts a step DOWN and nothing else. The band's own walk stops at any
height break of either sign, but the depth it reports is
max(0, reference - ground_past_the_step), so a step UP - a kerb, a wall, a
planter, a fence - reports exactly 0.0 and is read as "nothing to fall off".
It then receives EDGE_MARGIN (0.075 m) instead of the full chair half width,
and hazard_clearance() cannot see it at all. On this route that is 283 of 742
station edges. Driving into a raised kerb is not safer than driving off a
dropped one, so here every observed break is a hard boundary.

Usage: make_no_go_zones.py <band.json> <route.json> <out.json>
"""

import json
import math
import sys
from collections import Counter

import numpy as np


# The band searches +-6.0 m in 0.3 m bins, so a walk that reaches 5.7 m never
# found an edge: that is open ground, not an unmeasured one.
OPEN_GROUND_LIMIT_M = 5.7
SEVERE_DROP_M = 0.12
CHAIR_HALF_WIDTH_M = 0.35
BAND_MARGIN_M = 0.10
EDGE_MARGIN_M = 0.075
BAND_FLOOR_M = 0.15
# How far past a hard edge the no-go strip is drawn. Enough to be unmistakable
# in a viewer and to swallow the chair if it crossed; the edge itself is the
# boundary that matters.
NO_GO_DEPTH_M = 1.2
# The corridor width reported here is how far the chair's CENTRE may travel
# laterally, not how much room the chair has: the clearance subtracted from
# each edge already contains the half width plus its margin, so a centre
# freedom of 0.30 m still keeps both wheels BAND_MARGIN_M clear of both edges.
# Comparing it against the chair's own width would be a category error. What
# matters is that it stays positive, and that there is enough of it to steer.
TIGHT_CORRIDOR_M = 0.30
# Physical gap between the two edges. Below this the chair does not fit at all,
# clearance policy aside.
CHAIR_WIDTH_M = 2 * CHAIR_HALF_WIDTH_M
# A hazard this close to the recorded line is already under the wheel once the
# chair is allowed its shallow-edge clearance.
EDGE_AT_WHEEL_M = 0.45
# Stations up to this far apart are reported as one segment.
SEGMENT_GAP = 2


def classify(limit_m, drop_m, kind=None):
    """What kind of edge bounded the walk on one side of one station.

    The band now records this directly. The inference below is kept for bands
    generated before it did, where a raised edge is only distinguishable from
    open ground by its limit falling short of the search window.
    """

    if kind is not None:
        return kind
    if drop_m is None or drop_m < 0.0:
        return "unscanned"
    if drop_m >= SEVERE_DROP_M:
        return "drop"
    if drop_m > 0.0:
        return "lip"
    if limit_m >= OPEN_GROUND_LIMIT_M:
        return "open"
    return "step_up"


def is_hard(kind):
    """Boundaries the chair must not cross, whichever way the ground goes."""

    return kind in ("unscanned", "drop", "step_up")


def usable_limit(limit_m, kind):
    """Corrected lateral allowance for the chair centre on one side."""

    if is_hard(kind):
        # No floor toward a hazard: a negative value means the chair must sit
        # off the recorded line, toward the other side.
        return limit_m - (CHAIR_HALF_WIDTH_M + BAND_MARGIN_M)
    return max(limit_m - EDGE_MARGIN_M, BAND_FLOOR_M)


def load_band(path):
    data = json.load(open(path, encoding="utf-8"))
    stations = data["stations"]
    xy = np.array([[s["x"], s["y"]] for s in stations])
    heading = np.radians([s["heading_deg"] for s in stations])
    # Left normal, matching SafetyBand: positive lateral offset is to the left.
    normals = np.stack([-np.sin(heading), np.cos(heading)], axis=1)
    return data, stations, xy, normals


def edge_runs(stations, side):
    """Contiguous station ranges whose edge on one side is a hard boundary."""

    runs = []
    current = None
    for index, station in enumerate(stations):
        kind = classify(station[side + "_m"], station[side + "_drop_m"],
                        station.get(side + "_kind"))
        if is_hard(kind):
            if current is not None and index == current["end"] + 1:
                current["end"] = index
                current["kinds"].append(kind)
            else:
                if current is not None:
                    runs.append(current)
                current = {"start": index, "end": index, "kinds": [kind]}
        elif current is not None:
            runs.append(current)
            current = None
    if current is not None:
        runs.append(current)
    return runs


def strip_polygon(xy, normals, stations, side, start, end, depth_m):
    """Map-frame polygon covering the ground beyond a hard edge."""

    sign = 1.0 if side == "left" else -1.0
    inner, outer = [], []
    for index in range(start, end + 1):
        limit = stations[index][side + "_m"]
        base = xy[index] + normals[index] * (sign * limit)
        inner.append(base)
        outer.append(base + normals[index] * (sign * depth_m))
    ring = inner + outer[::-1]
    return [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in ring]


def drivable_polygon(xy, normals, left, right):
    """The corrected corridor, as one ribbon along the whole route."""

    left_edge = [xy[i] + normals[i] * left[i] for i in range(len(xy))]
    right_edge = [xy[i] - normals[i] * right[i] for i in range(len(xy))]
    ring = left_edge + right_edge[::-1]
    return [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in ring]


def risk_reasons(station, kind_left, kind_right, width_m):
    reasons = []
    if station["left_m"] + station["right_m"] < CHAIR_WIDTH_M:
        reasons.append("chair_does_not_fit")
    if width_m < 0.0:
        reasons.append("no_safe_position")
    elif width_m < TIGHT_CORRIDOR_M:
        reasons.append("tight_corridor")
    for side, kind in (("left", kind_left), ("right", kind_right)):
        if not is_hard(kind):
            continue
        if station[side + "_m"] < EDGE_AT_WHEEL_M:
            reasons.append("{}_{}_at_wheel".format(side, kind))
        elif kind == "unscanned":
            reasons.append("{}_unscanned".format(side))
    return reasons


def main(band_path, route_path, out_path):
    data, stations, xy, normals = load_band(band_path)
    route = json.load(open(route_path, encoding="utf-8"))

    kinds_left, kinds_right, left, right = [], [], [], []
    for station in stations:
        kl = classify(station["left_m"], station["left_drop_m"],
                      station.get("left_kind"))
        kr = classify(station["right_m"], station["right_drop_m"],
                      station.get("right_kind"))
        kinds_left.append(kl)
        kinds_right.append(kr)
        left.append(usable_limit(station["left_m"], kl))
        right.append(usable_limit(station["right_m"], kr))

    tally = Counter(kinds_left) + Counter(kinds_right)
    width = [left[i] + right[i] for i in range(len(stations))]

    risks = []
    for index, station in enumerate(stations):
        reasons = risk_reasons(
            station, kinds_left[index], kinds_right[index], width[index]
        )
        if reasons:
            risks.append((index, reasons))

    segments = []
    for index, reasons in risks:
        if segments and index - segments[-1]["end"] <= SEGMENT_GAP:
            segments[-1]["end"] = index
            segments[-1]["reasons"].update(reasons)
        else:
            segments.append(
                {"start": index, "end": index, "reasons": set(reasons)}
            )

    risk_zones = []
    for number, segment in enumerate(segments, start=1):
        start, end = segment["start"], segment["end"]
        widths = width[start:end + 1]
        risk_zones.append({
            "name": "risk_{:02d}".format(number),
            "reason": ", ".join(sorted(segment["reasons"])),
            "source": "generated",
            "station_range": [start, end],
            "length_m": end - start + 1,
            "min_corridor_width_m": round(min(widths), 2),
            "centre": [
                round(float(xy[(start + end) // 2][0]), 2),
                round(float(xy[(start + end) // 2][1]), 2),
            ],
        })

    no_go = []
    for side in ("left", "right"):
        for run in edge_runs(stations, side):
            kinds = Counter(run["kinds"])
            no_go.append({
                "side": side,
                "kind": kinds.most_common(1)[0][0],
                "station_range": [run["start"], run["end"]],
                "source": "generated",
                "polygon": strip_polygon(
                    xy, normals, stations, side,
                    run["start"], run["end"], NO_GO_DEPTH_M,
                ),
            })

    document = {
        "frame": data["frame"],
        "map_id": "merged_0707_0725_v1",
        "map_sha256": (
            "ee317581328d3eaeee86ba448b0068c1016ca1452664b6cdaba2d874320d0431"
        ),
        "route": route_path.split("/")[-1],
        "band": band_path.split("/")[-1],
        "station_spacing_m": data["station_spacing_m"],
        "chair_half_width_m": CHAIR_HALF_WIDTH_M,
        "policy": (
            "every observed height break is a hard boundary, whichever way "
            "the ground goes; the shipped band consumer counts downward steps "
            "only"
        ),
        "edge_tally": dict(tally),
        "drivable": {"polygon": drivable_polygon(xy, normals, left, right)},
        "risk_zones": risk_zones,
        "no_go": no_go,
        # Hand-added zones go here. Nothing generated is written into this
        # list, so regenerating the file cannot silently drop them.
        "manual_no_go": [],
    }
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=1)
        handle.write("\n")

    print("edges: " + ", ".join(
        "%s=%d" % (k, v) for k, v in sorted(tally.items())))
    print("corridor width: min %.2f  p10 %.2f  median %.2f m" % (
        min(width), float(np.percentile(width, 10)), float(np.median(width))))
    print("no-go strips: %d" % len(no_go))
    print("risk zones: %d covering %d stations" % (
        len(risk_zones), sum(z["length_m"] for z in risk_zones)))
    for zone in risk_zones:
        print("  %s  st %3d-%-3d  min %5.2f m  %s" % (
            zone["name"], zone["station_range"][0], zone["station_range"][1],
            zone["min_corridor_width_m"], zone["reason"]))
    print("wrote %s" % out_path)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2], sys.argv[3])
