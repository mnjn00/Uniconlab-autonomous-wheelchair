"""Deployment invariants for the ZIP-v5 corridor and exact centreline."""

import hashlib
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


ROUTE_PATH = ROOT / "routes" / "20260803_route_v5_waypoints.json"
BAND_PATH = ROOT / "routes" / "20260803_route_v5_safety_band.json"
SOURCE_DIR = ROOT / "data" / "route_corridor_v5"
ARCHIVE_SHA256 = (
    "6361ff648fab58c162b9860d6f6468155926a51d97256c306026d9be827d62c4"
)


def load_route(path):
    document = json.loads(path.read_text(encoding="utf-8"))
    points = np.array(
        [[waypoint["x"], waypoint["y"]] for waypoint in document["waypoints"]],
        dtype=float,
    )
    return document, points


def test_runtime_route_matches_v5_zip_dense_centreline():
    runtime, runtime_xy = load_route(ROUTE_PATH)
    source, source_xy = load_route(
        SOURCE_DIR / "route_centerline_waypoints_dense_0p2m.json"
    )

    assert runtime["count"] == source["waypoint_count"] == 2004
    assert np.array_equal(runtime_xy, source_xy)
    assert runtime["reference_point"] == "chair_centre"
    assert np.allclose(
        runtime["chair_centre_in_body_xyz"], CHAIR_CENTRE_IN_BODY_XYZ
    )
    assert runtime["operator_target_waypoint_index"] == 1120
    assert runtime["operator_target_xy_m"] == [156.159, -84.341]


def test_v5_route_points_and_segments_stay_in_runtime_band():
    _, points = load_route(ROUTE_PATH)
    band = SafetyBand(str(BAND_PATH))

    assert np.all(band.contains_many(points))
    assert np.all(band.contains_many(0.5 * (points[:-1] + points[1:])))
    assert np.all(band.left + band.right >= 0.0)


def test_v5_route_stays_in_zip_mask():
    _, points = load_route(ROUTE_PATH)
    metadata = json.loads(
        (SOURCE_DIR / "metadata.json").read_text(encoding="utf-8")
    )
    with Image.open(SOURCE_DIR / metadata["image"]) as image:
        mask = np.asarray(image) == 254
    height, width = mask.shape
    resolution = float(metadata["resolution_m_per_pixel"])
    origin_x, origin_y = metadata["origin"][:2]

    columns = np.floor((points[:, 0] - origin_x) / resolution).astype(int)
    rows_from_bottom = np.floor((points[:, 1] - origin_y) / resolution).astype(int)
    rows = height - 1 - rows_from_bottom

    assert np.all((columns >= 0) & (columns < width))
    assert np.all((rows >= 0) & (rows < height))
    assert np.all(mask[rows, columns])


def test_v5_band_records_drawn_authority_without_fake_drop_evidence():
    band = json.loads(BAND_PATH.read_text(encoding="utf-8"))
    field_source = json.loads(
        (
            SOURCE_DIR / "route_centerline_waypoints_field_0p5m.json"
        ).read_text(encoding="utf-8")
    )

    assert band["corridor"]["source"] == (
        "route_2d_map_v5.pgm + route_2d_map_v5.yaml"
    )
    assert band["corridor"]["stations_covered"] == len(band["stations"]) == 802
    assert [
        [station["x"], station["y"], station["heading_deg"]]
        for station in band["stations"]
    ] == [
        [waypoint["x"], waypoint["y"], waypoint["heading_deg"]]
        for waypoint in field_source["waypoints"]
    ]
    assert band["physical_edge_semantics"]["source"] == (
        "not present in route_2d_map_v5.zip"
    )
    assert all(
        station["left_kind"] == station["right_kind"] == "open"
        for station in band["stations"]
    )


def test_v5_source_provenance_is_hash_pinned():
    metadata = json.loads(
        (SOURCE_DIR / "metadata.json").read_text(encoding="utf-8")
    )
    image = SOURCE_DIR / metadata["image"]
    waypoints = SOURCE_DIR / metadata["source_waypoints"]
    band_stations = SOURCE_DIR / metadata["band_station_waypoints"]

    assert metadata["provenance"]["source_archive"] == "route_2d_map_v5.zip"
    assert metadata["provenance"]["source_archive_sha256"] == ARCHIVE_SHA256
    assert metadata["image_sha256"] == hashlib.sha256(image.read_bytes()).hexdigest()
    assert metadata["source_waypoints_sha256"] == hashlib.sha256(
        waypoints.read_bytes()
    ).hexdigest()
    assert metadata["band_station_waypoints_sha256"] == hashlib.sha256(
        band_stations.read_bytes()
    ).hexdigest()
