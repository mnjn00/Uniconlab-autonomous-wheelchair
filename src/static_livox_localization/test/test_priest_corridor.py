"""The band-to-corridor reading, checked against the shipped bands.

The planner treats its corridor as ground truth about where the chair may
be, so the one job here is that the reading never claims more room than the
band does - not after the hazard margins, not after the operator's drawing,
and not on either of the two very different bands this repository actually
ships (0727: measured edge kinds; v4: drawn corridor, no drop semantics).
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


def stations(path):
    return json.loads(path.read_text(encoding="utf-8"))["stations"]


@pytest.mark.parametrize("band", [BAND_0727, BAND_V4],
                         ids=["0727-measured", "v4-drawn"])
def test_the_corridor_never_claims_more_room_than_the_band(band):
    if not band.exists():
        pytest.skip("band not shipped")
    centres, normals, left, right = pc.corridor_arrays(str(band))
    raw = stations(band)

    assert len(centres) == len(raw)
    for index, station in enumerate(raw):
        assert left[index] <= station["left_m"] + 1e-9
        assert right[index] <= station["right_m"] + 1e-9
        assert left[index] >= 0.0 and right[index] >= 0.0


def test_the_drawn_corridor_narrows_where_it_is_present():
    """On v4 every station carries the drawing, and corridor_limit insets
    the chair half width from it - so the planning limit must sit at or
    inside the drawn value, never between the drawing and the raw edge."""
    if not BAND_V4.exists():
        pytest.skip("band not shipped")
    _, _, left, right = pc.corridor_arrays(str(BAND_V4))
    for index, station in enumerate(stations(BAND_V4)):
        assert left[index] <= station["left_corridor_m"] + 1e-9
        assert right[index] <= station["right_corridor_m"] + 1e-9


def test_hazard_kinds_cost_room_on_the_measured_band():
    """The 0727 band carries measured drop/step_up/lip edges. Wherever one
    exists, the planning limit must sit strictly inside the raw distance to
    the break - a corridor that plans right up to a kerb edge has spent the
    entire hazard margin before control error is even counted."""
    if not BAND_0727.exists():
        pytest.skip("band not shipped")
    _, _, left, right = pc.corridor_arrays(str(BAND_0727))
    raw = stations(BAND_0727)
    checked = 0
    for index, station in enumerate(raw):
        if station["left_kind"] in ("drop", "step_up", "lip") \
                and station["left_m"] > 0.0:
            assert left[index] < station["left_m"], index
            checked += 1
        if station["right_kind"] in ("drop", "step_up", "lip") \
                and station["right_m"] > 0.0:
            assert right[index] < station["right_m"], index
            checked += 1
    assert checked > 100, "the measured band stopped carrying hazard kinds"


def test_normals_point_left_of_the_station_heading():
    """Positive lateral offset must mean left, matching left_m/right_m.
    A flipped normal swaps the kerb side silently - the corridor stays the
    same width and every clearance is measured against the wrong edge."""
    if not BAND_V4.exists():
        pytest.skip("band not shipped")
    centres, normals, _, _ = pc.corridor_arrays(str(BAND_V4))
    for station, normal in zip(stations(BAND_V4)[::50], normals[::50]):
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
