from pathlib import Path

from route_audit import audit_route_bundle

REPO_ROOT = Path(__file__).resolve().parents[3]
BAND_PATH = REPO_ROOT / "routes/20260727_new_route_safety_band.json"
ROUTE_PATH = REPO_ROOT / "routes/20260727_new_route_waypoints.json"
SAFETY_BAND_PATH = (
    REPO_ROOT / "src/static_livox_localization/scripts/safety_band.py"
)


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
    assert len(audit.failed_station_chord_indices) == 42
    assert len(audit.failed_route_chord_indices) == 17
