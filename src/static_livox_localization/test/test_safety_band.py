"""Numeric tests for the safety band against the SHIPPED route band.

The band is the chair's only drop protection - the MID360 cannot see the
ground within ~2.4 m, so nothing else stops a wheel going off a kerb. These
tests execute the geometry against routes/aejimun_to_gongsen_safety_band.json
rather than asserting that substrings appear in the source.

The defect they exist to prevent: lateral_limits used to take the MORE
PERMISSIVE of the two bracketing stations, so one wide neighbour dilated a
narrow station by up to 35x.
"""

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parents[3]
sys.path.insert(0, str(ROOT / "src" / "static_livox_localization" / "scripts"))

from safety_band import (  # noqa: E402
    BAND_FLOOR, CHAIR_HALF_WIDTH, SafetyBand)

BAND_JSON = ROOT / "routes" / "aejimun_to_gongsen_safety_band.json"


def _band():
    return SafetyBand(str(BAND_JSON))


def _station_point(band, index, lateral):
    return band.xy[index] + band.normals[index] * lateral


def test_shipped_band_loads_and_matches_the_json():
    band = _band()
    raw = json.load(open(BAND_JSON))["stations"]
    assert len(band.xy) == len(raw) > 300
    assert len(band.left) == len(band.right) == len(raw)


def test_a_wide_neighbour_cannot_dilate_a_narrow_station():
    """The C2 regression: pick the station with the largest gap between its
    own usable limit and its neighbour's, and require containment to follow
    the restrictive one."""
    band = _band()
    worst, worst_gap = None, 0.0
    for i in range(1, len(band.left) - 1):
        neighbour = max(band.left[i - 1], band.left[i + 1])
        gap = neighbour - band.left[i]
        if gap > worst_gap:
            worst, worst_gap = i, gap
    assert worst is not None and worst_gap > 1.0, \
        "shipped band should contain at least one large neighbour gap"

    own_limit = band.left[worst]
    # a point just beyond this station's OWN limit must be rejected, even
    # though a neighbour would happily allow it
    point = _station_point(band, worst, own_limit + 0.25)
    assert not band.contains(point), (worst, own_limit, worst_gap)


def test_contains_agrees_with_the_limits_it_reports():
    """Self-consistency over the whole shipped band: contains() must be
    exactly the lateral_limits interval, with no hidden widening. Asserting
    against a station index instead would be wrong - a point offset from
    station i can legitimately be bracketed by different stations."""
    band = _band()
    for i in range(0, len(band.xy), 3):
        for offset in (-2.0, -0.8, -0.2, 0.0, 0.2, 0.8, 2.0):
            point = _station_point(band, i, offset)
            lateral, lo, hi = band.lateral_limits(point)
            assert band.contains(point) == (lo - 1e-6 <= lateral <= hi + 1e-6)


def test_reported_limits_never_exceed_the_bracketing_stations():
    """The property the old max() broke: the interval handed back must be
    the RESTRICTIVE one of the two stations the code itself selected."""
    band = _band()
    for i in range(0, len(band.xy), 3):
        for offset in (-2.0, -0.5, 0.0, 0.5, 2.0):
            point = _station_point(band, i, offset)
            d = np.linalg.norm(band.xy - point, axis=1)
            order = np.argsort(d)[:2]
            _lateral, lo, hi = band.lateral_limits(point)
            assert abs(hi - min(band.left[j] for j in order)) < 1e-12
            assert abs(lo + min(band.right[j] for j in order)) < 1e-12


def test_the_driven_line_is_always_contained():
    """Tightening the predicate must not make the route undrivable: the
    mapped driven line has to stay inside the band at every station."""
    band = _band()
    for i in range(len(band.xy)):
        assert band.contains(band.xy[i]), "station %d rejects its own centre" % i


def test_a_far_off_point_is_rejected_everywhere_on_the_route():
    """Whatever stations end up bracketing it, 6 m off the driven line must
    never be judged inside the band."""
    band = _band()
    for i in range(0, len(band.xy), 5):
        for side in (+1.0, -1.0):
            assert not band.contains(_station_point(band, i, side * 6.0))


def test_grace_widens_but_does_not_invert():
    band = _band()
    i = int(np.argmin(band.left))
    just_outside = _station_point(band, i, band.left[i] + 0.20)
    assert not band.contains(just_outside)
    assert band.contains(just_outside, grace=0.30)


def test_clamp_never_returns_a_point_outside_the_band():
    band = _band()
    for i in range(0, len(band.xy), 7):
        for lateral in (-6.0, -1.4, -1.0, 0.0, 1.0, 1.4, 6.0):
            clamped = band.clamp(_station_point(band, i, lateral))
            assert band.contains(clamped, grace=0.05), (i, lateral)


def test_floor_and_chair_width_are_actually_applied():
    band = _band()
    raw = json.load(open(BAND_JSON))["stations"]
    assert band.left.min() >= BAND_FLOOR - 1e-9
    # a station whose raw margin exceeds the floor must have the chair's
    # half width and the margin subtracted, not be passed through
    for i, s in enumerate(raw):
        if s["left_m"] > CHAIR_HALF_WIDTH + 0.6:
            assert band.left[i] < s["left_m"] - CHAIR_HALF_WIDTH + 1e-9
            break
    else:
        raise AssertionError("no station wide enough to check the subtraction")


def test_how_much_of_the_route_sits_on_the_floor_is_visible():
    """Not a pass/fail property but a guard on a known weakness: most of
    the route falls back to BAND_FLOOR because the generator cannot resolve
    a drop closer than ~0.6 m. If this ratio moves, the band data changed."""
    band = _band()
    on_floor = int(np.sum(np.isclose(band.left, BAND_FLOOR)))
    assert 0 < on_floor < len(band.left), on_floor


# --- route vs band: the facts the follower's chord check has to cope with ---

ROUTE_JSON = ROOT / "routes" / "aejimun_to_gongsen_waypoints.json"


def _route():
    return np.array([[w["x"], w["y"]] for w in
                     json.load(open(ROUTE_JSON))["waypoints"]])


def _chord_in_band(band, a, b, grace=0.0, spacing=0.25):
    span = float(np.linalg.norm(b - a))
    if span < 1e-6:
        return True
    steps = max(2, int(np.ceil(span / spacing)))
    return all(band.contains(a + (b - a) * (k / float(steps)), grace=grace)
               for k in range(1, steps + 1))


def test_the_shipped_route_is_not_band_safe():
    """Documents the root cause behind the follower's chord check, and fails
    loudly if the route is ever regenerated: 4 of 85 waypoints lie outside
    the band and 8 of 84 chords leave it, because the 353-station driven line
    was sparsified to 85 points without regard for the band."""
    band, route = _band(), _route()
    outside = [i for i in range(len(route)) if not band.contains(route[i])]
    leaving = sum(1 for i in range(len(route) - 1)
                  if not _chord_in_band(band, route[i], route[i + 1]))
    assert outside, "route unexpectedly band-safe - was it regenerated?"
    assert len(outside) <= 4, outside
    assert leaving <= 8, leaving


def test_a_short_chord_from_the_driven_line_is_always_in_band():
    """Why backing the lookahead off works: a ~1 m chord is safe everywhere,
    so a creep speed is always available."""
    band = _band()
    bad = 0
    for i in range(0, len(band.xy) - 2):
        direction = band.xy[i + 1] - band.xy[i]
        n = np.linalg.norm(direction)
        if n < 1e-6:
            continue
        target = band.xy[i] + direction / n * 0.9
        if not _chord_in_band(band, band.xy[i], target, grace=0.10):
            bad += 1
    assert bad <= 2, "%d of %d short chords left the band" % (bad, len(band.xy))


def test_a_long_chord_is_not_always_in_band():
    """The reason speed has to back off at all: a 3.4 m chord cannot follow
    curves tighter than roughly a 10 m radius inside a 0.15 m band."""
    band = _band()
    bad = 0
    for i in range(0, len(band.xy) - 5):
        direction = band.xy[i + 3] - band.xy[i]
        n = np.linalg.norm(direction)
        if n < 1e-6:
            continue
        target = band.xy[i] + direction / n * 3.4
        if not _chord_in_band(band, band.xy[i], target, grace=0.10):
            bad += 1
    assert bad > 0, "long chords unexpectedly all safe - check the band data"
