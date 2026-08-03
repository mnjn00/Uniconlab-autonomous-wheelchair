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
from priest_constraints import CANONICAL_FOOTPRINT  # noqa: E402
from priest_execution_safety import (  # noqa: E402
    oriented_footprint_contained,
)
from safety_band import SafetyBand  # noqa: E402


ROUTE_PATH = ROOT / "routes" / "20260803_route_v5_waypoints.json"
BAND_PATH = ROOT / "routes" / "20260803_route_v5_safety_band.json"
SOURCE_DIR = ROOT / "data" / "route_corridor_v5"
ARCHIVE_SHA256 = (
    "6361ff648fab58c162b9860d6f6468155926a51d97256c306026d9be827d62c4"
)


def test_runtime_route_matches_v5_zip_dense_centreline() -> None:
    # Given the committed ZIP source and the route selected for deployment.
    runtime = json.loads(ROUTE_PATH.read_text(encoding="utf-8"))
    source = json.loads((
        SOURCE_DIR / "route_centerline_waypoints_dense_0p2m.json"
    ).read_text(encoding="utf-8"))

    # When both are interpreted as map-frame centrelines.
    runtime_xy = np.array([
        [point["x"], point["y"]] for point in runtime["waypoints"]
    ])
    source_xy = np.array([
        [point["x"], point["y"]] for point in source["waypoints"]
    ])

    # Then deployment preserves every source point and the operator target.
    assert runtime["count"] == source["waypoint_count"] == 2004
    assert np.array_equal(runtime_xy, source_xy)
    assert runtime["reference_point"] == "chair_centre"
    assert np.allclose(
        runtime["chair_centre_in_body_xyz"], CHAIR_CENTRE_IN_BODY_XYZ)
    assert runtime["operator_target_waypoint_index"] == 1120
    assert runtime["operator_target_xy_m"] == [156.159, -84.341]


def test_v5_route_and_footprint_stay_in_runtime_band() -> None:
    # Given the exact deployed route and its runtime band.
    route = json.loads(ROUTE_PATH.read_text(encoding="utf-8"))
    points = np.array([
        [point["x"], point["y"]] for point in route["waypoints"]
    ])
    yaw = np.radians([point["yaw_deg"] for point in route["waypoints"]])
    band = SafetyBand(str(BAND_PATH))

    # When containment is checked at points, chords, and oriented poses.
    midpoints = 0.5 * (points[:-1] + points[1:])
    footprints = oriented_footprint_contained(band, points, yaw)

    # Then every commanded route sample remains admissible.
    assert np.all(band.contains_many(points))
    assert np.all(band.contains_many(midpoints))
    assert np.all(footprints)


def test_v5_physical_footprint_stays_inside_zip_mask() -> None:
    # Given the ZIP mask frame and the canonical physical chair footprint.
    metadata = json.loads((SOURCE_DIR / "metadata.json").read_text(
        encoding="utf-8"))
    route = json.loads(ROUTE_PATH.read_text(encoding="utf-8"))
    with Image.open(SOURCE_DIR / metadata["image"]) as image:
        mask = np.asarray(image) == 254
    centres = np.array([
        [point["x"], point["y"]] for point in route["waypoints"]
    ])
    yaw = np.radians([point["yaw_deg"] for point in route["waypoints"]])
    footprint = CANONICAL_FOOTPRINT
    body_corners = np.array([
        [footprint.front_m, footprint.half_width_m],
        [footprint.front_m, -footprint.half_width_m],
        [-footprint.rear_m, footprint.half_width_m],
        [-footprint.rear_m, -footprint.half_width_m],
    ])

    # When each oriented footprint corner is projected into the source mask.
    cosine, sine = np.cos(yaw), np.sin(yaw)
    rotation = np.stack([
        np.stack([cosine, -sine], axis=1),
        np.stack([sine, cosine], axis=1),
    ], axis=1)
    corners = centres[:, None, :] + np.einsum(
        "nij,kj->nki", rotation, body_corners)
    height, width = mask.shape
    resolution = float(metadata["resolution_m_per_pixel"])
    origin_x, origin_y = metadata["origin"][:2]
    flat = corners.reshape(-1, 2)
    columns = np.floor((flat[:, 0] - origin_x) / resolution).astype(int)
    rows_from_bottom = np.floor(
        (flat[:, 1] - origin_y) / resolution).astype(int)
    rows = height - 1 - rows_from_bottom

    # Then no corner leaves the operator-authored drivable mask.
    assert np.all((columns >= 0) & (columns < width))
    assert np.all((rows >= 0) & (rows < height))
    assert np.all(mask[rows, columns])


def test_v5_band_records_drawn_authority_without_fake_drop_evidence() -> None:
    # Given the generated runtime band.
    band = json.loads(BAND_PATH.read_text(encoding="utf-8"))
    field_source = json.loads((
        SOURCE_DIR / "route_centerline_waypoints_field_0p5m.json"
    ).read_text(encoding="utf-8"))

    # When its authority and station evidence are inspected.
    stations = band["stations"]

    # Then it identifies the ZIP mask and never invents measured edge kinds.
    assert band["corridor"]["source"] == (
        "route_2d_map_v5.pgm + route_2d_map_v5.yaml")
    assert band["corridor"]["stations_covered"] == len(stations) == 802
    assert [
        [station["x"], station["y"], station["heading_deg"]]
        for station in stations
    ] == [
        [point["x"], point["y"], point["heading_deg"]]
        for point in field_source["waypoints"]
    ]
    assert band["physical_edge_semantics"]["source"] == (
        "not present in route_2d_map_v5.zip")
    assert all(
        station["left_kind"] == station["right_kind"] == "open"
        for station in stations)


def test_v5_band_extents_reproduce_the_zip_mask_boundary() -> None:
    # Given the recorded 1 cm generation rule and every runtime station.
    metadata = json.loads((SOURCE_DIR / "metadata.json").read_text(
        encoding="utf-8"))
    band = json.loads(BAND_PATH.read_text(encoding="utf-8"))
    with Image.open(SOURCE_DIR / metadata["image"]) as image:
        mask = np.asarray(image) == 254
    stations = band["stations"]
    centres = np.array([[item["x"], item["y"]] for item in stations])
    headings = np.radians([item["heading_deg"] for item in stations])
    normals = np.stack([-np.sin(headings), np.cos(headings)], axis=1)
    height, width = mask.shape
    resolution = float(metadata["resolution_m_per_pixel"])
    origin_x, origin_y = metadata["origin"][:2]

    def inside(points: np.ndarray) -> np.ndarray:
        columns = np.floor(
            (points[:, 0] - origin_x) / resolution).astype(int)
        rows_from_bottom = np.floor(
            (points[:, 1] - origin_y) / resolution).astype(int)
        rows = height - 1 - rows_from_bottom
        valid = (
            (columns >= 0) & (columns < width)
            & (rows >= 0) & (rows < height)
        )
        result = np.zeros(len(points), dtype=bool)
        result[valid] = mask[rows[valid], columns[valid]]
        return result

    # Then each stored endpoint is inside and the next 1 cm sample is out,
    # except where the documented 6 m lateral-search ceiling was reached.
    for side, sign in (("left", 1.0), ("right", -1.0)):
        extents = np.array([item[f"{side}_m"] for item in stations])
        endpoints = centres + sign * extents[:, None] * normals
        next_samples = centres + sign * (extents + 0.01)[:, None] * normals
        limited = extents < 6.0
        assert np.all(inside(endpoints))
        assert np.all(~inside(next_samples[limited]))
    assert all(
        station["left_m"] == station["left_corridor_m"]
        and station["right_m"] == station["right_corridor_m"]
        for station in stations)


def test_v5_source_provenance_is_hash_pinned() -> None:
    # Given the compact committed source bundle.
    metadata = json.loads((SOURCE_DIR / "metadata.json").read_text(
        encoding="utf-8"))
    image = SOURCE_DIR / metadata["image"]
    waypoints = SOURCE_DIR / metadata["source_waypoints"]
    band_stations = SOURCE_DIR / metadata["band_station_waypoints"]

    # When the committed artifacts are independently hashed.
    image_digest = hashlib.sha256(image.read_bytes()).hexdigest()
    waypoint_digest = hashlib.sha256(waypoints.read_bytes()).hexdigest()
    station_digest = hashlib.sha256(band_stations.read_bytes()).hexdigest()

    # Then the receipt binds the user ZIP and both runtime source artifacts.
    assert metadata["provenance"]["source_archive"] == "route_2d_map_v5.zip"
    assert metadata["provenance"]["source_archive_sha256"] == ARCHIVE_SHA256
    assert metadata["image_sha256"] == image_digest
    assert metadata["source_waypoints_sha256"] == waypoint_digest
    assert metadata["band_station_waypoints_sha256"] == station_digest
