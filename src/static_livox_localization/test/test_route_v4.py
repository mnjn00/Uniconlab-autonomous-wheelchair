"""Deployment invariants for the ZIP-v4 corridor and exact centreline."""

import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image


ROOT = Path(__file__).parents[3]
SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"
sys.path.insert(0, str(SCRIPTS))

from body_frame import CHAIR_CENTRE_IN_BODY_XYZ  # noqa: E402
from safety_band import SafetyBand  # noqa: E402


ROUTE_PATH = ROOT / "routes" / "20260802_route_v4_waypoints.json"
BAND_PATH = ROOT / "routes" / "20260802_route_v4_safety_band.json"
SOURCE_DIR = ROOT / "data" / "route_corridor_v4"


def load_route(path):
    doc = json.loads(path.read_text(encoding="utf-8"))
    points = np.array([[w["x"], w["y"]] for w in doc["waypoints"]],
                      dtype=float)
    return doc, points


def test_runtime_route_is_the_zip_dense_centreline_exactly():
    runtime, runtime_xy = load_route(ROUTE_PATH)
    source, source_xy = load_route(
        SOURCE_DIR / "route_centerline_waypoints_dense_0p2m.json")

    assert runtime["count"] == source["waypoint_count"] == 2002
    assert np.array_equal(runtime_xy, source_xy)
    assert runtime["reference_point"] == "chair_centre"
    assert np.allclose(runtime["chair_centre_in_body_xyz"],
                       CHAIR_CENTRE_IN_BODY_XYZ)


def test_route_points_and_segments_stay_in_the_runtime_band():
    _, points = load_route(ROUTE_PATH)
    band = SafetyBand(str(BAND_PATH))

    assert np.all(band.contains_many(points))
    midpoints = 0.5 * (points[:-1] + points[1:])
    assert np.all(band.contains_many(midpoints))
    assert np.all(band.left + band.right >= 0.0)


def test_runtime_route_stays_in_the_zip_mask():
    _, points = load_route(ROUTE_PATH)
    meta = json.loads((SOURCE_DIR / "metadata.json").read_text(
        encoding="utf-8"))
    mask = np.array(Image.open(SOURCE_DIR / meta["image"])) == 254
    height, width = mask.shape
    resolution = float(meta["resolution_m_per_pixel"])
    origin_x, origin_y = meta["origin"][:2]

    col = np.floor((points[:, 0] - origin_x) / resolution).astype(int)
    row_bottom = np.floor((points[:, 1] - origin_y) / resolution).astype(int)
    row = height - 1 - row_bottom

    assert np.all((col >= 0) & (col < width))
    assert np.all((row >= 0) & (row < height))
    assert np.all(mask[row, col])


def test_zip_mask_authority_is_explicit_not_fake_drop_measurement():
    doc = json.loads(BAND_PATH.read_text(encoding="utf-8"))

    assert doc["corridor"]["source"] == (
        "route_2d_map_v4.pgm + route_2d_map_v4.yaml")
    assert doc["physical_edge_semantics"]["source"] == (
        "not present in route_2d_map_v4.zip")
    assert all(s["left_kind"] == "open" and s["right_kind"] == "open"
               for s in doc["stations"])
