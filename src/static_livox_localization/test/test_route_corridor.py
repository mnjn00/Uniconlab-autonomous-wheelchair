"""Numeric tests for the hand-drawn corridor against the SHIPPED 0727 band.

The corridor is operator judgement drawn over a map. The band is measurement.
Everything here exists to keep the first from ever overruling the second.

Two defects these pin, both found while wiring it up:

  - corridor_limit floored its RESULT at zero. usable_limit deliberately
    returns a negative limit toward a measured kerb - the edge is inside the
    driven line and the chair must sit off it - so the floor handed those
    stations back clearance the map says is not there. The floor belongs on
    the corridor term alone, with min() last.
  - Clamping one side to zero while the other was negative left stations
    with no admissible lateral position at all. Three of 381 on this route
    became new stops, created by a drawing rather than by any measurement.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).parents[3]
sys.path.insert(0, str(ROOT / "src" / "static_livox_localization" / "scripts"))

from safety_band import (  # noqa: E402
    CHAIR_HALF_WIDTH, SafetyBand, corridor_limit, usable_limit)

BAND_JSON = ROOT / "routes" / "20260727_chair_centred_safety_band.json"
AUDIT_JSON = ROOT / "routes" / "20260727_chair_centred_corridor_audit.json"
ROUTE_JSON = ROOT / "routes" / "20260727_chair_centred_waypoints.json"
# waypoint_follower.OFF_BAND_GRACE
GRACE = 0.10


@pytest.fixture(scope="module")
def stations():
    return json.load(open(BAND_JSON))["stations"]


@pytest.fixture(scope="module")
def band():
    return SafetyBand(str(BAND_JSON))


def _measured(s):
    return (usable_limit(s["left_m"], s.get("left_drop_m"), s.get("left_kind")),
            usable_limit(s["right_m"], s.get("right_drop_m"), s.get("right_kind")))


def test_the_shipped_band_actually_carries_a_corridor(stations):
    covered = [s for s in stations if "left_corridor_m" in s]
    assert len(stations) > 300
    assert 0 < len(covered) < len(stations), \
        "the 0727 drawing covers most of the route but not all of it"
    for s in covered:
        assert "right_corridor_m" in s, "a covered station needs both sides"
        assert s["left_corridor_m"] >= 0.0 and s["right_corridor_m"] >= 0.0


def test_an_absent_drawing_changes_nothing():
    assert corridor_limit(2.45, None) == 2.45
    assert corridor_limit(-0.2, None) == -0.2


def test_the_corridor_can_only_narrow():
    # wider than the measured limit: ignored
    assert corridor_limit(0.6, 5.0) == pytest.approx(0.6)
    # narrower: binds, inset by the chair half width
    assert corridor_limit(2.45, 0.95) == pytest.approx(0.60)


def test_the_floor_is_on_the_corridor_term_not_the_result():
    """The regression: max(0, min(usable, corridor - half)) loosened a kerb.

    A station whose measured limit is -0.20 m has a kerb inside the driven
    line. Whatever the drawing says there, the answer may not become 0.0.
    """
    assert corridor_limit(-0.20, 0.30) == pytest.approx(-0.20)
    assert corridor_limit(-0.20, 5.00) == pytest.approx(-0.20)
    # and the corridor's own contribution still never goes negative
    assert corridor_limit(1.00, 0.10) == pytest.approx(0.0)


def test_no_station_is_widened_by_the_drawing(stations, band):
    for i, s in enumerate(stations):
        ml, mr = _measured(s)
        assert band.left[i] <= ml + 1e-9, f"station {i} left widened"
        assert band.right[i] <= mr + 1e-9, f"station {i} right widened"


def test_the_drawing_creates_no_station_with_nowhere_to_be(stations, band):
    """A crossed band means the chair holds. Measurement may say that; a
    drawing may not. Stations where it would are reverted and flagged."""
    measured_empty = sum(1 for s in stations if sum(_measured(s)) < 0.0)
    shipped_empty = int(((band.left + band.right) < 0.0).sum())
    assert shipped_empty <= measured_empty
    for i in np.nonzero(band.corridor_yielded)[0]:
        ml, mr = _measured(stations[i])
        assert band.left[i] == pytest.approx(ml)
        assert band.right[i] == pytest.approx(mr)


def test_hazard_geometry_is_untouched(stations, band):
    """hazard_clearance paces the chair and safe_offset leans it away from
    kerbs. Both read the measured edge, so the drawing must not reach them -
    a corridor written into left_m would have told the speed policy a 2.45 m
    kerb was 0.80 m away for most of the route."""
    for i, s in enumerate(stations):
        assert band.edge_left[i] == pytest.approx(s["left_m"])
        assert band.edge_right[i] == pytest.approx(s["right_m"])
        assert band.narrow[i] == (s["left_m"] + s["right_m"] < 1.2)


def test_the_drawing_actually_binds(stations, band):
    """Otherwise this whole path is inert and nobody would notice."""
    narrowed = sum(1 for i, s in enumerate(stations)
                   if band.left[i] < _measured(s)[0] - 1e-9
                   or band.right[i] < _measured(s)[1] - 1e-9)
    assert narrowed > 100, "the 0727 drawing narrows most of the route"
    open_side = np.array([s["left_m"] for s in stations])
    assert np.median(open_side) > 2.0, \
        "the measured band is permissive on the open side - that is the point"
    assert np.median(band.left + band.right) < np.median(open_side)


def test_the_driven_line_survives_the_clamp(band):
    """Two complete autonomous runs on 2026-07-31 drove this line. A drawing
    that refuses it is a drawing error, not a new safety rule."""
    route = json.load(open(ROUTE_JSON))
    xy = np.array([[w["x"], w["y"]] for w in route["waypoints"]])
    contained = sum(1 for p in xy if band.contains(p, grace=GRACE))
    assert contained / len(xy) > 0.91


def test_batch_containment_is_identical_to_scalar_containment(band):
    """Perception must read the follower's exact band, not an approximation."""
    route = json.load(open(ROUTE_JSON))
    xy = np.array([[w["x"], w["y"]] for w in route["waypoints"]])
    probes = np.vstack((xy[::17], xy[::17] + np.array([0.8, -0.6])))
    for grace in (0.0, GRACE):
        scalar = np.array([band.contains(point, grace=grace)
                           for point in probes])
        assert np.array_equal(band.contains_many(probes, grace=grace), scalar)


def test_the_audit_is_present_and_agrees_with_the_band(stations):
    """The uncovered stations and the route excursions are the part a person
    has to look at. An audit that has drifted from the band is worse than
    none, because it is the record someone will act on."""
    audit = json.load(open(AUDIT_JSON))
    assert audit["stations_total"] == len(stations)
    covered = sum(1 for s in stations if "left_corridor_m" in s)
    assert audit["stations_covered"] == covered
    assert set(audit["stations_uncovered"]) == {
        i for i, s in enumerate(stations) if "left_corridor_m" not in s}
    assert audit["usable_limit_m"]["never_widened"] is True
    assert audit["admissible_width_m"]["no_new_empty_stations"] is True
    assert audit["route_outside_corridor"]["excursions"], \
        "the 0727 drawing does leave the driven line in places; say so"


def test_the_corridor_asset_is_pinned():
    """The band is derived from this image. A silently swapped drawing is a
    silently different set of limits on the chair."""
    import hashlib
    meta = json.load(open(ROOT / "data" / "route_corridor_v3" / "metadata.json"))
    image = ROOT / "data" / "route_corridor_v3" / meta["image"]
    digest = hashlib.sha256(image.read_bytes()).hexdigest()
    assert digest == meta["image_sha256"]
    assert meta["map_sha256"] == json.load(
        open(ROOT / "routes" / "20260727_chair_centred_no_go_zones.json"))["map_sha256"]
    assert meta["authority"]["may_widen_the_map_derived_band"] is False
