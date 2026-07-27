"""Per-station drop-free lateral limits along the route (map frame).

This is the wheelchair's ONLY drop protection. The MID360 cannot see the
ground within ~2.4 m (vertical FOV -7 deg at a 0.3 m mount), so there is no
live kerb or drop detection at all - containment inside this band, derived
offline from the prior map, is what keeps a wheel on the pavement.

Deliberately free of ROS imports so the geometry can be unit-tested against
the shipped band JSON; see test/test_safety_band.py.
"""

import json

import numpy as np

CHAIR_HALF_WIDTH = 0.35
BAND_MARGIN = 0.10
# the driven line itself is proven safe, so never shrink the usable band
# below this; narrow stations creep instead of holding
BAND_FLOOR = 0.15
NARROW_BAND_WIDTH = 1.2
# clamp re-evaluates because moving a point changes which stations
# bracket it; two passes converge on the shipped band
CLAMP_MAX_PASSES = 3


class SafetyBand:
    def __init__(self, path):
        data = json.load(open(path))
        self.xy = np.array([[s["x"], s["y"]] for s in data["stations"]])
        heading = np.radians([s["heading_deg"] for s in data["stations"]])
        self.normals = np.stack([-np.sin(heading), np.cos(heading)], axis=1)
        usable_left, usable_right, narrow = [], [], []
        for s in data["stations"]:
            usable_left.append(
                max(s["left_m"] - CHAIR_HALF_WIDTH - BAND_MARGIN, BAND_FLOOR))
            usable_right.append(
                max(s["right_m"] - CHAIR_HALF_WIDTH - BAND_MARGIN, BAND_FLOOR))
            narrow.append(s["left_m"] + s["right_m"] < NARROW_BAND_WIDTH)
        self.left = np.array(usable_left)
        self.right = np.array(usable_right)
        self.narrow = np.array(narrow)

    def lateral_limits(self, point):
        """Signed cross-track offset and the limits that bracket it.

        The limit is the MORE RESTRICTIVE of the two nearest stations. Taking
        the more permissive one (which this did) let a single wide neighbour
        dilate a narrow station enormously: measured on the shipped band,
        station 70's own usable limit is 0.15 m while its neighbour's is
        5.70 m, so containment passed out to 5.25 m - 35x the real limit,
        with the mapped drop 0.30 m away. Bracketing stations are 1 m apart,
        so the chair is always inside the span of both and must satisfy both.
        """
        d = np.linalg.norm(self.xy - point, axis=1)
        order = np.argsort(d)[:2]
        k = int(order[0])
        lateral = float(np.dot(point - self.xy[k], self.normals[k]))
        lo = -min(self.right[j] for j in order)
        hi = min(self.left[j] for j in order)
        return lateral, lo, hi

    def contains(self, point, grace=0.0):
        lateral, lo, hi = self.lateral_limits(point)
        return lo - grace - 1e-6 <= lateral <= hi + grace + 1e-6

    def is_narrow(self, point):
        d = np.linalg.norm(self.xy - point, axis=1)
        return bool(self.narrow[int(np.argmin(d))])

    def chord_is_contained(self, start, target, grace=0.0, spacing=0.25):
        """Require every sampled point on a straight drive chord to be safe."""
        start = np.asarray(start, dtype=float)
        target = np.asarray(target, dtype=float)
        span = float(np.linalg.norm(target - start))
        if span < 1e-6:
            return self.contains(target, grace=grace)
        steps = max(2, int(np.ceil(span / spacing)))
        return all(self.contains(
            start + (target - start) * (float(k) / steps), grace=grace)
            for k in range(1, steps + 1))

    def clamp(self, point):
        """Pull a point back inside the band, and verify it landed inside.

        Moving the point changes which stations bracket it, so a single pass
        can hand back something the containment test then rejects - clamp
        approving a target that contains() refuses is exactly the mismatch
        that let the follower steer toward a point the hold logic considered
        out of band. Re-clamping at the new location converges in one or two
        passes; if it somehow does not, fall back to the station centre,
        which is on the driven line and therefore always inside.
        """
        current = np.asarray(point, dtype=float)
        for _ in range(CLAMP_MAX_PASSES):
            d = np.linalg.norm(self.xy - current, axis=1)
            k = int(np.argmin(d))
            lateral, lo, hi = self.lateral_limits(current)
            clamped = min(max(lateral, lo), hi)
            candidate = self.xy[k] + self.normals[k] * clamped
            if self.contains(candidate):
                return candidate
            current = candidate
        return self.xy[int(np.argmin(
            np.linalg.norm(self.xy - np.asarray(point, dtype=float),
                           axis=1)))]
