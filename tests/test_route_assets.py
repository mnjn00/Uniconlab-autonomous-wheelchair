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
