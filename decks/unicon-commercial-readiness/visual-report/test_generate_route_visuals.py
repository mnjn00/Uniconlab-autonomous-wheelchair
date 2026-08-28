#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "numpy==2.4.6",
#   "pillow==12.3.0",
#   "pydantic==2.13.4",
#   "pytest==9.1.1",
# ]
# ///

import json
import sys
from pathlib import Path

import pytest

# This report generator is an explicitly isolated uv/Python-3.11 tool. ROS
# Noetic on Ubuntu 20.04 is Python 3.8, where dataclass(slots=True) and
# hashlib.file_digest are unavailable. The Noetic repository suite must not
# fail while merely collecting a tool whose own metadata excludes that
# interpreter; its dedicated uv invocation remains authoritative.
if sys.version_info < (3, 11):
    pytest.skip(
        "route visual report requires Python >=3.11 as declared by the script",
        allow_module_level=True,
    )

from generate_route_visuals import digest_file
from route_audit import audit_route_bundle

REPO_ROOT = Path(__file__).resolve().parents[3]
BAND_PATH = REPO_ROOT / "routes/20260727_new_route_safety_band.json"
ROUTE_PATH = REPO_ROOT / "routes/20260727_new_route_waypoints.json"
SAFETY_BAND_PATH = REPO_ROOT / "src/static_livox_localization/scripts/safety_band.py"
RECEIPT_PATH = Path(__file__).resolve().parent / "assets/route-band-audit.json"


def test_runtime_predicate_reproduces_known_0727_failures() -> None:
    audit = audit_route_bundle(BAND_PATH, ROUTE_PATH, SAFETY_BAND_PATH)

    assert audit.station_count == 371
    assert audit.route_waypoint_count == 75
    assert audit.rejected_station_indices == (
        97,
        98,
        99,
        100,
        106,
        107,
        108,
        111,
        112,
        113,
        114,
        125,
        126,
        143,
        144,
        145,
        146,
        147,
        148,
        149,
        150,
        202,
        203,
        204,
        206,
        207,
        210,
        211,
        212,
        213,
        286,
        287,
        288,
        369,
        370,
    )
    assert audit.inverted_station_indices == (97, 98)
    assert audit.failed_station_chord_indices == (
        96,
        97,
        98,
        99,
        105,
        106,
        107,
        108,
        110,
        111,
        112,
        113,
        114,
        124,
        125,
        140,
        141,
        142,
        143,
        144,
        145,
        146,
        147,
        148,
        149,
        151,
        152,
        201,
        202,
        203,
        205,
        206,
        209,
        210,
        211,
        212,
        213,
        285,
        286,
        287,
        368,
        369,
    )
    assert audit.rejected_route_waypoint_indices == (
        10,
        18,
        19,
        20,
        26,
        37,
        38,
        39,
        74,
    )
    assert audit.failed_route_chord_indices == (
        9,
        17,
        18,
        19,
        20,
        22,
        24,
        25,
        26,
        36,
        37,
        38,
        39,
        46,
        55,
        66,
        73,
    )


def test_committed_receipt_matches_internal_sources_and_outputs() -> None:
    payload = json.loads(RECEIPT_PATH.read_text(encoding="utf-8"))
    provenance = payload["provenance"]
    expected = {
        "generator_source": Path(__file__).resolve().parent
        / "generate_route_visuals.py",
        "band_json": BAND_PATH,
        "route_json": ROUTE_PATH,
        "safety_band_source": SAFETY_BAND_PATH,
        "overview_png": Path(__file__).resolve().parent
        / "assets/route-band-overview.png",
        "hotspot_sheet_png": Path(__file__).resolve().parent
        / "assets/route-band-hotspots.png",
    }
    for key, path in expected.items():
        assert provenance[key]["sha256"] == digest_file(path).sha256
    assert provenance["canonical_ply"] == {
        "path": "/Volumes/무제/merged_0707_0725_v1/mergedmap.ply",
        "bytes": 594_886_982,
        "sha256": "3639f5942101e67d8f62baf533017475146ebb681f4a8482ecaf0f2a7cec6536",
    }
    assert provenance["canonical_ply_points"] == 37_180_425
    assert provenance["runtime_pcd"] == {
        "path": ("/Volumes/무제/merged_0707_0725_v1/merged_0707_0725_0p20m_xyzi.pcd"),
        "bytes": 43_141_936,
        "sha256": "ee317581328d3eaeee86ba448b0068c1016ca1452664b6cdaba2d874320d0431",
    }
    assert provenance["runtime_pcd_points"] == 2_696_359
    for tile in provenance["hotspot_tiles"]:
        assert tile["sha256"] == digest_file(REPO_ROOT / tile["path"]).sha256
    for image in provenance["web_assets"]:
        assert image["sha256"] == digest_file(REPO_ROOT / image["path"]).sha256


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
