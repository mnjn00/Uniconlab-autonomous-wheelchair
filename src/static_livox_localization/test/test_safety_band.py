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

import safety_band  # noqa: E402
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


ROUTE_JSON = ROOT / "routes" / "aejimun_to_gongsen_waypoints.json"


def _route():
    return np.array([[w["x"], w["y"]] for w in
                     json.load(open(ROUTE_JSON))["waypoints"]])


def test_shipped_route_requires_chord_guarding():
    band, route = _band(), _route()
    outside = [i for i, point in enumerate(route) if not band.contains(point)]
    leaving = [i for i in range(len(route) - 1)
               if not band.chord_is_contained(route[i], route[i + 1])]
    assert outside == [39, 54, 66, 67]
    assert leaving == [38, 41, 53, 65, 66, 67]


def test_short_driven_line_chords_are_contained_with_operational_grace():
    band = _band()
    failures = 0
    for i in range(len(band.xy) - 2):
        direction = band.xy[i + 1] - band.xy[i]
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            continue
        target = band.xy[i] + direction / norm * 0.9
        failures += not band.chord_is_contained(
            band.xy[i], target, grace=0.10)
    assert failures <= 2


def _lookahead_point(route, index, pose, distance):
    projections = []
    for i in range(max(0, index - 1), min(len(route) - 1, index + 1)):
        delta = route[i + 1] - route[i]
        fraction = np.clip(
            np.dot(pose - route[i], delta) / np.dot(delta, delta), 0.0, 1.0)
        point = route[i] + fraction * delta
        projections.append((np.linalg.norm(pose - point), i, point))
    _, segment_index, start = min(projections, key=lambda item: item[0])
    for i in range(segment_index, len(route) - 1):
        end = route[i + 1]
        segment = np.linalg.norm(end - start)
        if segment >= 1e-6:
            if distance <= segment:
                return start + (end - start) * (distance / segment)
            distance -= segment
        start = end
    return route[-1]


def test_baseline_lookahead_backs_off_only_at_the_measured_tight_station():
    band, route = _band(), _route()
    unsafe = []
    for i, station in enumerate(band.xy):
        index = int(np.argmin(np.linalg.norm(route - station, axis=1)))
        target = band.clamp(_lookahead_point(route, index, station, 1.8))
        if not band.chord_is_contained(station, target, grace=0.10):
            unsafe.append(i)
            backed_off = band.clamp(
                _lookahead_point(route, index, station, 1.4))
            assert band.chord_is_contained(
                station, backed_off, grace=0.10)
    assert unsafe == [201]


def test_clearance_is_chosen_from_the_measured_drop_not_applied_uniformly():
    """A shallow lip must not cost the same 0.45 m of usable width as a kerb
    into a roadway - that inset on both sides is what left this route too
    narrow to step around a pedestrian."""
    full = safety_band.CHAIR_HALF_WIDTH + safety_band.BAND_MARGIN

    assert safety_band.edge_clearance(0.0) == safety_band.EDGE_MARGIN
    assert safety_band.edge_clearance(
        safety_band.DROP_SEVERE_M - 0.01) == safety_band.EDGE_MARGIN
    assert safety_band.edge_clearance(safety_band.DROP_SEVERE_M) == full
    assert safety_band.edge_clearance(0.25) == full
    assert safety_band.EDGE_MARGIN < full


def test_an_unseen_edge_is_treated_as_a_drop_not_as_open_ground():
    """No returns past the limit means the scan could not see what is
    there. Absence of evidence must not read as evidence of flat ground."""
    full = safety_band.CHAIR_HALF_WIDTH + safety_band.BAND_MARGIN
    assert safety_band.edge_clearance(-1.0) == full


def test_a_band_without_drop_fields_keeps_the_conservative_inset():
    """Bands generated before depth measurement existed must behave exactly
    as they were validated, not silently inherit the tighter margin."""
    full = safety_band.CHAIR_HALF_WIDTH + safety_band.BAND_MARGIN
    assert safety_band.edge_clearance(None) == full


def test_no_floor_is_granted_toward_a_measured_drop():
    """BAND_FLOOR exists because the driven line is proven passable, which
    says nothing about ground 15 cm to its side. Applied toward a kerb it
    granted the outer wheel 0.20 m past a 24 cm drop at 32 stations of the
    2026-07-27 route."""
    kerb_at, drop = 0.30, 0.24
    limit = safety_band.usable_limit(kerb_at, drop)
    assert limit < safety_band.BAND_FLOOR
    wheel = limit + safety_band.CHAIR_HALF_WIDTH
    assert wheel <= kerb_at, (
        "wheel reaches %.2f m with the kerb at %.2f m" % (wheel, kerb_at))


def test_the_floor_still_applies_where_there_is_nothing_to_fall_off():
    assert safety_band.usable_limit(0.20, 0.0) == safety_band.BAND_FLOOR


def test_a_legacy_band_without_depths_keeps_its_validated_floor():
    """Withdrawing the floor retroactively would turn every edge of a
    pre-depth band into an unpassable one."""
    assert safety_band.usable_limit(0.20, None) == safety_band.BAND_FLOOR


def test_leaning_off_the_line_only_happens_next_to_a_hazard():
    """The recorded line is the only path known to have been driven, so
    leaving it needs a reason, and the shift is bounded."""
    band = _band()
    offsets = np.array([band.safe_offset(p) for p in band.xy])
    assert np.abs(offsets).max() <= safety_band.BIAS_MAX + 1e-9


def test_leaning_increases_clearance_from_the_kerb():
    band = _band()
    for point in band.xy:
        before = band.hazard_clearance(point)
        if not np.isfinite(before):
            continue
        after = band.hazard_clearance(band.recentre(point))
        assert after >= before - 1e-6, (
            "re-centring moved the chair closer to a hazard: %.3f -> %.3f"
            % (before, after))


def test_speed_slack_reacts_to_the_kerb_not_to_the_band_width():
    """A corridor pinched between two kerbs and one with a kerb on a single
    side are different situations; keying speed on total width spent the
    same caution on both."""
    band = _band()
    for point in band.xy[:40]:
        _, lo, hi = band.lateral_limits(point)
        width = hi - lo
        clearance = band.hazard_clearance(point)
        if np.isfinite(clearance):
            assert clearance != width
