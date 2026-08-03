"""The shipped safety band, read as a corridor the PRIEST planner can hold.

The band and the planner mean different things by "lateral limit", and the
gap between them is exactly where a kerb would slip through. A station's raw
left_m is the distance to where the ground breaks - the last place a wheel
could physically be, not a place to plan through. usable_limit() subtracts
the hazard margin appropriate to the edge's measured kind, and
corridor_limit() narrows further to the operator's drawing where one exists.
Both already live in safety_band.py because the follower applies the same
policy; this module exists so the planner cannot end up with a private,
slightly different notion of where the band ends.

On the v4 band every edge kind is "open" and every drop is zero - the ZIP
carried no drop measurement, and the band says so itself under
physical_edge_semantics. usable_limit() therefore subtracts nothing there,
and the whole constraint is the operator's drawing. That is the shipped
reality, not a choice made here; it is recorded in the README's warning and
re-measuring it is an open item. This module simply refuses to make it look
better than it is.

Normals point LEFT of the station heading (heading rotated +90 degrees), so
a positive lateral offset is a move to the left - the same convention the
band's left_m/right_m already use.
"""

import json
import math

import numpy as np

from safety_band import corridor_limit, edge_clearance


def station_limits(station):
    """(left, right) planning limits for one station, in metres.

    The measured limit is reduced by the hazard margin for its kind, then by
    the drawn corridor where present.

    Deliberately NOT usable_limit(). That function carries BAND_FLOOR, which
    guarantees 0.15 m on a non-severe edge because the driven line itself is
    proven passable and the follower only ever sits on or near that line.
    The planner has no such restraint - a limit is room it will actually
    use - so on a station where the band measured 0.05 m to a lip, the floor
    would hand the planner 0.10 m of ground the map never offered. Here the
    hazard margin is always paid and there is no floor; edge_clearance() is
    still the shared policy for how much a given edge kind costs.

    Clamped at zero rather than allowed negative: a station whose width
    vanishes still has its centreline, and the corridor constraint
    degenerates to "stay on the line" instead of becoming an empty set that
    poisons every horizon containing it. The follower's own band containment
    still applies at runtime as the backstop.
    """
    left = station["left_m"] - edge_clearance(
        station.get("left_drop_m", 0.0), station.get("left_kind"))
    right = station["right_m"] - edge_clearance(
        station.get("right_drop_m", 0.0), station.get("right_kind"))
    if "left_corridor_m" in station:
        left = corridor_limit(left, station["left_corridor_m"])
    if "right_corridor_m" in station:
        right = corridor_limit(right, station["right_corridor_m"])
    return max(float(left), 0.0), max(float(right), 0.0)


def corridor_arrays(band_path):
    """(centres, normals, left, right) from a shipped band file.

    Arrays in station order, ready for priest_planner.Corridor. Raises on a
    band too short to define a direction of travel - a one-station corridor
    has no arc length, and arc length is how the planner knows which way the
    goal lies.
    """
    with open(band_path) as handle:
        stations = json.load(handle)["stations"]
    if len(stations) < 2:
        raise ValueError(
            "band %s has %d stations; a corridor needs at least two to have "
            "a direction" % (band_path, len(stations)))

    centres = np.array([[s["x"], s["y"]] for s in stations], dtype=np.float64)
    headings = np.radians([float(s["heading_deg"]) for s in stations])
    normals = np.stack([-np.sin(headings), np.cos(headings)], axis=1)
    limits = [station_limits(s) for s in stations]
    left = np.array([l for l, _ in limits], dtype=np.float64)
    right = np.array([r for _, r in limits], dtype=np.float64)
    return centres, normals, left, right
