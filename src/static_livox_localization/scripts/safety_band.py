"""Per-station drop-free lateral limits along the route (map frame).

This is the wheelchair's ONLY drop protection. The MID360 cannot see the
ground within ~2.4 m (vertical FOV -7 deg at a 0.3 m mount), so there is no
live kerb or drop detection at all - containment inside this band, derived
offline from the prior map, is what keeps a wheel on the pavement.

Clearance is chosen per edge from the measured depth of the step that
bounded it, not applied uniformly:

  - A real kerb into a road (>= DROP_SEVERE_M) keeps the full
    CHAIR_HALF_WIDTH + BAND_MARGIN inset, so the outer wheel physically
    cannot reach the edge.
  - A shallow lip, a gentle camber change, or open ground keeps only
    EDGE_MARGIN. A uniform half-width inset costs 0.45 m of usable width
    on BOTH sides; on this route that left several stretches too narrow
    for the chair to step around a pedestrian or a parked obstacle, so it
    would stop and wait instead of passing. Trading clearance for
    passability is only sound where the map shows nothing to fall off,
    which is what the per-edge depth establishes.
  - An edge the scan could not see past (depth < 0, no returns) is
    treated as severe. Absence of data is not evidence of flat ground.

A station may additionally carry left_corridor_m / right_corridor_m, the
operator's hand-drawn corridor (tools/apply_route_corridor_mask.py). That
narrows the usable limits and nothing else. hazard_clearance, safe_offset
and is_narrow keep reading the measured fields, because they answer
physical questions - how far is the fall, which way is away from it, does
the chair fit - and a drawing is not evidence about any of them. In
particular is_narrow still gates on the MEASURED width, so the corridor
cannot make the chair creep down a stretch the map says is open.

Deliberately free of ROS imports so the geometry can be unit-tested against
the shipped band JSON; see test/test_safety_band.py.
"""

import json

import numpy as np

CHAIR_HALF_WIDTH = 0.35
BAND_MARGIN = 0.10
# Clearance kept from an edge with no real drop behind it. Small on
# purpose: it is what buys the lateral room to pass obstacles.
EDGE_MARGIN = 0.075
# A step at least this deep is treated as a fall hazard and gets the full
# chair-half-width inset. Below it the wheel rides over a lip instead of
# dropping off; standard kerbs into a roadway are 0.10-0.20 m.
DROP_SEVERE_M = 0.12
# the driven line itself is proven safe, so never shrink the usable band
# below this; narrow stations creep instead of holding
BAND_FLOOR = 0.15
NARROW_BAND_WIDTH = 1.2
# Cap on how far the planned line may be shifted from the recorded one.
# The recorded line is the only path known to have been driven, so leaving
# it is justified by moving away from a hazard and by nothing else.
BIAS_MAX = 0.5
# clamp re-evaluates because moving a point changes which stations
# bracket it; two passes converge on the shipped band
CLAMP_MAX_PASSES = 3
# The safe-side lean has the same re-bracketing hazard as clamp, so it is
# verified against the hazard geometry and backed off until it holds.
RECENTRE_MAX_PASSES = 4
RECENTRE_MIN_SHIFT = 0.02


def is_severe(drop_m, kind=None):
    """True where crossing the edge ends badly, either way the ground goes.

    `kind` is what the band generator actually saw - drop, step_up, lip,
    open or unscanned - and is authoritative when present. Without it the
    only evidence is the depth, which measures how far the ground FALLS and
    reports exactly 0.0 for a kerb, wall or planter that rises. That read a
    raised edge as open pavement and gave it EDGE_MARGIN: 283 of 742 edges
    on the 0727 route, every one of them 0.075 m of clearance from
    something the chair cannot drive over.
    """
    if kind is not None:
        return kind in ("drop", "step_up", "unscanned")
    return drop_m is None or drop_m < 0.0 or drop_m >= DROP_SEVERE_M


def edge_clearance(drop_m, kind=None):
    """Inset to keep from an edge bounded by a step of `drop_m` / `kind`.

    Bands generated before depth or kind existed carry neither; those are
    treated as severe, so an old band keeps exactly the conservative
    behaviour it was validated with.
    """
    if is_severe(drop_m, kind):
        return CHAIR_HALF_WIDTH + BAND_MARGIN
    return EDGE_MARGIN


def usable_limit(raw_m, drop_m, kind=None):
    """How far the chair centre may sit from the driven line on one side.

    BAND_FLOOR exists because the driven line itself is proven passable, so
    a station whose computed limit collapses should still allow a little
    room rather than holding. That reasoning only holds for ground the
    chair may actually occupy: applied toward a kerb it grants clearance
    the map says is not there. Measured on the 2026-07-27 route before the
    edge positions were refined, it permitted the outer wheel 0.20 m past a
    24 cm drop at 32 stations. Toward a fall hazard there is therefore no
    floor - the limit is whatever the kerb leaves, even if that is negative,
    which simply means the chair must sit off the line toward the other
    side.
    """
    strict = raw_m - edge_clearance(drop_m, kind)
    if (kind is not None or drop_m is not None) and is_severe(drop_m, kind):
        # a MEASURED hazard: no floor toward it
        return strict
    # No depth field at all means a band generated before depths were
    # measured. Those keep the floor, and therefore exactly the behaviour
    # they were validated with - withdrawing it retroactively would turn
    # every legacy edge into an unpassable one.
    return max(strict, BAND_FLOOR)


def corridor_limit(usable_m, corridor_m):
    """Narrow a measured limit to the operator's hand-drawn corridor.

    The measured band says how far the chair CAN go before the ground
    breaks; on open ground that is often 2.45 m, and the follower will use
    it to step around a pedestrian. The drawing says how far it SHOULD go.
    Only the second is being applied here, and only in one direction:

      - It never widens. A drawing cannot authorise ground the map says
        breaks, so a corridor wider than the measured limit is ignored.
      - CHAIR_HALF_WIDTH is inset because the corridor was drawn for the
        chair, not for the point it turns about. No BAND_MARGIN on top:
        the corridor edge is a judgement, not a fall, and the fall margin
        is already carried by the measured limit this is a min() against.
      - The corridor's own contribution never goes negative. The driven
        line is the only path known to have been driven; a drawing that
        excludes it is a drawing error, and holding the corridor term at
        zero keeps the line legal so the chair reproduces the run instead
        of stopping. apply_route_corridor_mask.py's audit is where that
        gets found and fixed.

    The zero floor is applied to the corridor term ALONE, never to the
    result. usable_limit deliberately returns a NEGATIVE limit toward a
    measured fall - the kerb is inside the line and the chair must sit off
    it - and flooring the result at zero would hand that station back the
    0.20 m of clearance the map says is not there. min() last is what keeps
    the drawing incapable of loosening a kerb.

    `corridor_m` is None for a station the drawing does not cover, which is
    72 of 381 on the 0727 route. Those keep the measured limit untouched -
    an absent drawing is not a statement about the ground.
    """
    if corridor_m is None:
        return usable_m
    return min(usable_m, max(0.0, corridor_m - CHAIR_HALF_WIDTH))


class SafetyBand:
    def __init__(self, path):
        data = json.load(open(path))
        self.xy = np.array([[s["x"], s["y"]] for s in data["stations"]])
        heading = np.radians([s["heading_deg"] for s in data["stations"]])
        self.normals = np.stack([-np.sin(heading), np.cos(heading)], axis=1)
        usable_left, usable_right, narrow = [], [], []
        edge_left, edge_right, sev_left, sev_right = [], [], [], []
        corridor_yielded = []
        for s in data["stations"]:
            kind_left = s.get("left_kind")
            kind_right = s.get("right_kind")
            measured_l = usable_limit(s["left_m"], s.get("left_drop_m"), kind_left)
            measured_r = usable_limit(s["right_m"], s.get("right_drop_m"), kind_right)
            drawn_l = corridor_limit(measured_l, s.get("left_corridor_m"))
            drawn_r = corridor_limit(measured_r, s.get("right_corridor_m"))
            # A station whose limits cross has nowhere the chair may be, and
            # the follower holds there. The measurement is allowed to say
            # that - a kerb inside the line is a fact. A drawing is not, so
            # where the corridor would create the condition and the measured
            # band did not, the whole station reverts to the measurement and
            # is reported. Measured on the 0727 route this fires at 3 of 381
            # stations; without it they became three new stops.
            yielded = (drawn_l + drawn_r < 0.0) and (measured_l + measured_r >= 0.0)
            corridor_yielded.append(yielded)
            usable_left.append(measured_l if yielded else drawn_l)
            usable_right.append(measured_r if yielded else drawn_r)
            narrow.append(s["left_m"] + s["right_m"] < NARROW_BAND_WIDTH)
            edge_left.append(s["left_m"])
            edge_right.append(s["right_m"])
            sev_left.append(is_severe(s.get("left_drop_m"), kind_left))
            sev_right.append(is_severe(s.get("right_drop_m"), kind_right))
        self.left = np.array(usable_left)
        self.right = np.array(usable_right)
        self.narrow = np.array(narrow)
        self.edge_left = np.array(edge_left)
        self.edge_right = np.array(edge_right)
        self.severe_left = np.array(sev_left)
        self.severe_right = np.array(sev_right)
        self.corridor_yielded = np.array(corridor_yielded)

    def route_centre_clearance_violations(
            self, required_side_m=CHAIR_HALF_WIDTH + BAND_MARGIN,
            endpoint_guard=2):
        start = min(endpoint_guard, len(self.xy))
        end = max(start, len(self.xy) - endpoint_guard)
        bad = np.logical_or(
            self.edge_left[start:end] < required_side_m,
            self.edge_right[start:end] < required_side_m)
        return (np.nonzero(bad)[0] + start).tolist()

    def route_centre_chord_violations(
            self, endpoint_guard=2, spacing=0.1):
        start = min(endpoint_guard, max(0, len(self.xy) - 1))
        end = max(start, len(self.xy) - endpoint_guard - 1)
        return [
            index
            for index in range(start, end)
            if not self.chord_is_contained(
                self.xy[index],
                self.xy[index + 1],
                spacing=spacing)
        ]

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

    def margins_many(self, points):
        """Lateral offset and the limits bracketing it, per point.

        Exactly the geometry containment already computes, returned instead
        of thresholded. A planner that wants to prefer the middle of the
        corridor needs the distance to each edge, and the alternative -
        recomputing station lookup and normals on its side - is a second
        copy of the band rules that drifts from this one.
        """
        array = np.asarray(points, dtype=float)
        if array.ndim != 2 or array.shape[1] != 2:
            raise ValueError("points must have shape (N, 2)")
        if not len(array):
            empty = np.zeros(0, dtype=float)
            return empty, empty.copy(), empty.copy()

        delta = array[:, None, :] - self.xy[None, :, :]
        distance_sq = np.einsum("nsi,nsi->ns", delta, delta)
        if len(self.xy) == 1:
            order = np.zeros((len(array), 1), dtype=int)
        else:
            # Only the two nearest stations matter. A full 381-element sort
            # here consumed almost half of the perception node's 200 ms
            # cycle when forty 96-point clusters were present.
            order = np.argpartition(distance_sq, 1, axis=1)[:, :2]
            pair_distance = np.take_along_axis(distance_sq, order, axis=1)
            swap = pair_distance[:, 0] > pair_distance[:, 1]
            order[swap] = order[swap, ::-1]
        nearest = order[:, 0]
        lateral = np.einsum(
            "ni,ni->n", array - self.xy[nearest], self.normals[nearest])
        lo = -np.min(self.right[order], axis=1)
        hi = np.min(self.left[order], axis=1)
        return lateral, lo, hi

    @staticmethod
    def contained(lateral, lo, hi, grace=0.0):
        """The containment verdict, from margins already computed.

        Split out of contains_many so a caller that needs BOTH the verdict
        and the margins pays for the geometry once. dwa_core scores how near
        the corridor's edge each rollout runs as well as rejecting the ones
        that leave it, and asking contains_many and then margins_many ran the
        802-station nearest-neighbour search twice over the same 1,785
        points: 24.4 ms each on the target NUC against a 100 ms control
        period, so 96 % of the DWA cycle was one answer computed twice.

        The threshold lives here rather than at that caller for the reason
        margins_many exists at all - a consumer that writes out its own
        comparison is a second copy of the band rules, and the tolerance and
        the sign convention are part of those rules.
        """
        return ((lateral >= lo - grace - 1e-6) &
                (lateral <= hi + grace + 1e-6))

    def contains_many(self, points, grace=0.0):
        """Vectorised containment for map-frame ``(x, y)`` points.

        Object perception has dozens of clusters and each cluster has many
        returns. Calling contains() once per return would repeat the same
        381-station nearest-neighbour search thousands of times at 5 Hz. This
        is exactly the scalar geometry above, evaluated in one NumPy batch so
        the perception node can ask which returns lie in the driven corridor
        without maintaining a second, drifting copy of the band rules.
        """
        lateral, lo, hi = self.margins_many(points)
        if not len(lateral):
            return np.zeros(0, dtype=bool)
        return self.contained(lateral, lo, hi, grace)

    def hazard_clearance(self, point):
        """Distance from the nearest WHEEL to the nearest fall hazard.

        This, not the band's total width, is what a speed policy should
        react to. A 1.4 m band with a kerb on one side and open pavement on
        the other is not the same situation as 1.4 m pinched between two
        kerbs, and slowing identically for both spends most of the caution
        where there is nothing to fall off. Returns a large number where no
        severe edge brackets the point.
        """
        d = np.linalg.norm(self.xy - point, axis=1)
        order = np.argsort(d)[:2]
        k = int(order[0])
        lateral = float(np.dot(point - self.xy[k], self.normals[k]))
        gaps = []
        for j in order:
            if self.severe_left[j]:
                gaps.append(self.edge_left[j] - lateral - CHAIR_HALF_WIDTH)
            if self.severe_right[j]:
                gaps.append(lateral + self.edge_right[j] - CHAIR_HALF_WIDTH)
        return min(gaps) if gaps else float("inf")

    def safe_offset(self, point):
        """Lateral position that puts the most distance between the wheels
        and the mapped hazards, capped and kept inside the usable band.

        Defined from the hazard geometry, not from the middle of the usable
        interval: BAND_FLOOR and the per-edge clearance rule both distort
        that interval, so its midpoint is not where the chair is furthest
        from a fall. With hazards both sides the best position is midway
        between them; with one, move away from it as far as the band and
        the cap allow.
        """
        d = np.linalg.norm(self.xy - point, axis=1)
        order = np.argsort(d)[:2]
        _, lo, hi = self.lateral_limits(point)
        if hi < lo:
            # no admissible lateral position: report the least-bad one and
            # let containment decide whether to hold
            return 0.5 * (lo + hi)
        # Bracketing stations are 1 m apart and the chair lies between
        # them, so the binding edge is the nearer of the two - the same
        # pair, and the same min, that lateral_limits and
        # hazard_clearance use. Optimising against the nearest station
        # alone moved the chair toward a neighbour's closer kerb.
        left_bad = any(self.severe_left[j] for j in order)
        right_bad = any(self.severe_right[j] for j in order)
        left_edge = min(self.edge_left[j] for j in order)
        right_edge = min(self.edge_right[j] for j in order)
        if left_bad and right_bad:
            ideal = 0.5 * (left_edge - right_edge)
        elif left_bad:
            # move away from the kerb, but not past the point where the
            # opposite edge becomes the closer one
            ideal = max(-BIAS_MAX, 0.5 * (left_edge - right_edge))
        elif right_bad:
            ideal = min(BIAS_MAX, 0.5 * (left_edge - right_edge))
        else:
            # nothing to lean away from; stay on the proven line
            return 0.0
        ideal = min(max(ideal, -BIAS_MAX), BIAS_MAX)
        return min(max(ideal, lo), hi)

    def recentre(self, point):
        """Shift a point laterally onto its safe_offset, but only if that
        actually helps.

        Moving the point changes which stations bracket it, so the position
        that looked best against one pair can be measured against another:
        on the shipped band, leaning 0.50 m off station 300 handed the point
        to station 299, whose kerb is 2.4 m nearer, and clearance fell from
        3.55 m to 1.55 m. The shift is therefore verified after the fact and
        halved until it is an improvement, falling back to not moving at
        all - the recorded line is always an acceptable answer.
        """
        base = self.hazard_clearance(point)
        if not np.isfinite(base):
            return point
        d = np.linalg.norm(self.xy - point, axis=1)
        k = int(np.argmin(d))
        lateral = float(np.dot(point - self.xy[k], self.normals[k]))
        shift = self.safe_offset(point) - lateral
        for _ in range(RECENTRE_MAX_PASSES):
            if abs(shift) < RECENTRE_MIN_SHIFT:
                break
            candidate = point + self.normals[k] * shift
            if self.contains(candidate) and \
                    self.hazard_clearance(candidate) >= base - 1e-9:
                return candidate
            shift *= 0.5
        return np.asarray(point, dtype=float)

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
