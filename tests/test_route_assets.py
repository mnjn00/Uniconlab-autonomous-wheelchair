import importlib.util
import hashlib
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).parents[1]
SCRIPT = (
    ROOT / "src" / "static_livox_localization" / "scripts" / "route_assets.py"
)
SPEC = importlib.util.spec_from_file_location("route_assets", SCRIPT)
ASSETS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ASSETS)
SAFETY_SPEC = importlib.util.spec_from_file_location(
    "safety_band",
    ROOT / "src" / "static_livox_localization" / "scripts" / "safety_band.py",
)
SAFETY = importlib.util.module_from_spec(SAFETY_SPEC)
SAFETY_SPEC.loader.exec_module(SAFETY)


def test_shipped_v6_v8_assets_are_cryptographically_bound():
    binding = ASSETS.validate_asset_binding(
        ROOT / "routes" / "20260812_route_v6_v8_waypoints.json",
        ROOT / "routes" / "20260812_route_v6_v8_safety_band.json",
        ROOT / "routes" / "route_2d_map_v8.yaml",
    )
    assert binding["route_id"].startswith("v6-v8:")
    band = SAFETY.SafetyBand(
        ROOT / "routes" / "20260812_route_v6_v8_safety_band.json"
    )
    assert band.route_centre_clearance_violations() == []
    assert band.route_centre_chord_violations() == []


def test_algorithm_default_assets_are_cryptographically_bound_and_clear():
    # The shipped default since 2026-08-16. It replaced the algorithm
    # route, which has 13 corners below TURN_FLOOR_SPEED and ended the
    # 08-15 drive at station 395; this one clears the curvature gate at
    # zero blocked stations.
    route = ROOT / "routes" / "20260815_route_v6_v8_trim_waypoints.json"
    band_path = (ROOT / "routes"
                 / "20260815_route_v6_v8_trim_safety_band.json")
    mask = ROOT / "routes" / "route_2d_map_v8.yaml"
    binding = ASSETS.validate_asset_binding(route, band_path, mask)
    assert binding["route_id"].startswith("v6-v8-t")
    route_data = json.loads(route.read_text(encoding="utf-8"))
    band_data = json.loads(band_path.read_text(encoding="utf-8"))
    assert route_data["count"] == 1886
    assert route_data["reference_point"] == "chair_centre"
    assert band_data["route_id"] == binding["route_id"]
    band = SAFETY.SafetyBand(band_path)
    assert band.route_centre_clearance_violations() == []
    assert band.route_centre_chord_violations() == []


def test_band_mismatch_fails_closed(tmp_path):
    route = ROOT / "routes" / "20260812_route_v6_v8_waypoints.json"
    band = json.loads(
        (ROOT / "routes" / "20260812_route_v6_v8_safety_band.json")
        .read_text(encoding="utf-8")
    )
    band["route_id"] = "wrong"
    changed = tmp_path / "changed-band.json"
    changed.write_text(json.dumps(band), encoding="utf-8")
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        ASSETS.validate_asset_binding(route, changed)


def test_route_waypoint_mutation_fails_closed(tmp_path):
    route = json.loads(
        (ROOT / "routes" / "20260812_route_v6_v8_waypoints.json")
        .read_text(encoding="utf-8")
    )
    route["waypoints"][100]["x"] += 1000.0
    changed = tmp_path / "changed-route.json"
    changed.write_text(json.dumps(route), encoding="utf-8")
    with pytest.raises(ValueError, match="route content SHA-256 mismatch"):
        ASSETS.validate_asset_binding(
            changed,
            ROOT / "routes" / "20260812_route_v6_v8_safety_band.json",
        )


def test_mask_geometry_metadata_mismatch_fails_closed(tmp_path):
    route = ROOT / "routes" / "20260812_route_v6_v8_waypoints.json"
    band = ROOT / "routes" / "20260812_route_v6_v8_safety_band.json"
    source = ROOT / "routes" / "route_2d_map_v8.yaml"
    changed = tmp_path / "route_2d_map_v8.yaml"
    changed.write_text(
        source.read_text(encoding="utf-8").replace(
            "resolution: 0.05", "resolution: 0.10"
        ),
        encoding="utf-8",
    )
    (tmp_path / "route_2d_map_v8.pgm").write_bytes(
        (ROOT / "routes" / "route_2d_map_v8.pgm").read_bytes()
    )
    with pytest.raises(ValueError, match="metadata SHA-256 mismatch"):
        ASSETS.validate_asset_binding(route, band, changed)


def test_provenance_binds_rebuild_inputs():
    provenance = json.loads(
        (ROOT / "routes" / "20260812_route_v6_v8_provenance.json")
        .read_text(encoding="utf-8")
    )
    expected = provenance["source_sha256"]
    paths = {
        "preferred_v6_pgm": ROOT / "routes" / "route_2d_map_v6.pgm",
        "drivable_v8_pgm": ROOT / "routes" / "route_2d_map_v8.pgm",
        "seed_v6_route": ROOT / "routes" / "20260812_route_v6_waypoints.json",
        "seed_v6_band": ROOT / "routes" / "20260812_route_v6_safety_band.json",
    }
    for name, path in paths.items():
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected[name]
