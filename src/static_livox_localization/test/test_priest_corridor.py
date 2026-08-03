"""The band-to-corridor adapter, checked against the shipped bands.

The planner's proposal corridor and the runtime band must be identical,
including deliberate BAND_FLOOR behavior and negative/crossed measured limits.
Only the runtime predicate is authoritative at final certification.
"""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT / "scripts"
BAND_0727 = ROOT.parents[1] / "routes" / "20260727_chair_centred_safety_band.json"
BAND_V4 = ROOT.parents[1] / "routes" / "20260802_route_v4_safety_band.json"
BAND_V5 = ROOT.parents[1] / "routes" / "20260803_route_v5_safety_band.json"


def load(name):
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            name, SCRIPTS / ("%s.py" % name))
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


pc = load("priest_corridor")
pt = load("priest_types")
sb = load("safety_band")


def stations(path):
    return json.loads(path.read_text(encoding="utf-8"))["stations"]


def write_measured_band(path: Path, left_m: float, right_m: float) -> Path:
    station = {
        "heading_deg": 0.0,
        "left_m": left_m,
        "right_m": right_m,
        "left_drop_m": 0.20,
        "right_drop_m": 0.20,
        "left_kind": "drop",
        "right_kind": "drop",
    }
    path.write_text(json.dumps({"stations": [
        dict(station, x=0.0, y=0.0),
        dict(station, x=1.0, y=0.0),
    ]}), encoding="utf-8")
    return path


def test_negative_measured_limit_matches_runtime_band_without_zero_floor(
        tmp_path: Path) -> None:
    band_path = write_measured_band(tmp_path / "negative.json", 0.30, 1.00)
    runtime_band = sb.SafetyBand(str(band_path))

    _, _, left, right = pc.corridor_arrays(str(band_path))

    assert np.array_equal(left, runtime_band.left), (
        "planner floored a negative measured limit that runtime preserves")
    assert np.array_equal(right, runtime_band.right)
    assert left[0] < 0.0


def test_crossed_measured_limits_remain_an_empty_runtime_corridor(
        tmp_path: Path) -> None:
    band_path = write_measured_band(tmp_path / "crossed.json", 0.30, 0.30)
    runtime_band = sb.SafetyBand(str(band_path))

    _, _, left, right = pc.corridor_arrays(str(band_path))

    assert np.array_equal(left, runtime_band.left)
    assert np.array_equal(right, runtime_band.right)
    assert np.all(left + right < 0.0), (
        "planner turned a crossed, empty runtime band into a legal centreline")


@pytest.mark.parametrize("band", [BAND_0727, BAND_V4, BAND_V5],
                         ids=["0727-measured", "v4-drawn", "v5-drawn"])
def test_the_corridor_exactly_matches_the_runtime_band(band):
    if not band.exists():
        pytest.skip("band not shipped")
    centres, normals, left, right = pc.corridor_arrays(str(band))
    raw = stations(band)
    runtime_band = sb.SafetyBand(str(band))

    assert len(centres) == len(raw)
    assert np.array_equal(left, runtime_band.left)
    assert np.array_equal(right, runtime_band.right)


def test_the_drawn_corridor_narrows_where_it_is_present():
    """On v5 every station carries the drawing, and corridor_limit insets
    the chair half width from it - so the planning limit must sit at or
    inside the drawn value, never between the drawing and the raw edge."""
    if not BAND_V5.exists():
        pytest.skip("band not shipped")
    _, _, left, right = pc.corridor_arrays(str(BAND_V5))
    for index, station in enumerate(stations(BAND_V5)):
        assert left[index] <= station["left_corridor_m"] + 1e-9
        assert right[index] <= station["right_corridor_m"] + 1e-9


def test_severe_hazard_kinds_cost_room_on_the_measured_band():
    """Severe measured edges never receive the legacy/open-ground floor."""
    if not BAND_0727.exists():
        pytest.skip("band not shipped")
    _, _, left, right = pc.corridor_arrays(str(BAND_0727))
    raw = stations(BAND_0727)
    checked = 0
    for index, station in enumerate(raw):
        if station["left_kind"] in ("drop", "step_up", "unscanned") \
                and station["left_m"] > 0.0:
            assert left[index] < station["left_m"], index
            checked += 1
        if station["right_kind"] in ("drop", "step_up", "unscanned") \
                and station["right_m"] > 0.0:
            assert right[index] < station["right_m"], index
            checked += 1
    assert checked > 0, "the measured band stopped carrying severe hazards"


def test_normals_point_left_of_the_station_heading():
    """Positive lateral offset must mean left, matching left_m/right_m.
    A flipped normal swaps the kerb side silently - the corridor stays the
    same width and every clearance is measured against the wrong edge."""
    if not BAND_V5.exists():
        pytest.skip("band not shipped")
    centres, normals, _, _ = pc.corridor_arrays(str(BAND_V5))
    for station, normal in zip(stations(BAND_V5)[::50], normals[::50]):
        heading = math.radians(station["heading_deg"])
        forward = np.array([math.cos(heading), math.sin(heading)])
        assert abs(float(np.dot(normal, forward))) < 1e-9
        left_of = np.array([-forward[1], forward[0]])
        assert float(np.dot(normal, left_of)) == pytest.approx(1.0)
    assert np.allclose(np.linalg.norm(normals, axis=1), 1.0)


def test_a_band_too_short_to_have_a_direction_is_refused(tmp_path):
    stub = tmp_path / "band.json"
    stub.write_text(json.dumps({"stations": [
        {"x": 0.0, "y": 0.0, "heading_deg": 0.0, "left_m": 1.0,
         "right_m": 1.0, "left_drop_m": 0.0, "right_drop_m": 0.0}]}))
    with pytest.raises(ValueError):
        pc.corridor_arrays(str(stub))


def test_arc_progress_is_continuous_inside_the_final_station_cell() -> None:
    corridor = pt.Corridor(
        np.array([[0.0, 0.0], [1.0, 0.0]]),
        np.array([[0.0, 1.0], [0.0, 1.0]]),
        np.ones(2), np.ones(2))

    assert corridor.arc_of(np.array([0.94, 0.02])) == pytest.approx(0.94)
    assert corridor.length_m - corridor.arc_of(np.array([0.94, 0.02])) \
        == pytest.approx(0.06)
