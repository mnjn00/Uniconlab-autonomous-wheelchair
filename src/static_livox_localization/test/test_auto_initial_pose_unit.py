import importlib.util
import json
import math
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "scripts" / "initial_pose_candidates.py"
GLOBAL_MODULE_PATH = ROOT / "scripts" / "initial_pose_global_search.py"


def load_module():
    assert MODULE_PATH.exists(), "known-start candidate module is not implemented"
    spec = importlib.util.spec_from_file_location(
        "initial_pose_candidates_under_test", MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_global_module():
    assert GLOBAL_MODULE_PATH.exists(), "global fallback module is not implemented"
    spec = importlib.util.spec_from_file_location(
        "initial_pose_global_search_under_test", GLOBAL_MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.path.insert(0, str(ROOT / "scripts"))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    return module


def write_route(path, **overrides):
    document = {
        "frame": "map",
        "body_frame_profile": "builtin",
        "count": 1,
        "waypoints": [
            {"x": 5.15, "y": 1.13, "z": -0.12, "yaw_deg": 14.2},
        ],
    }
    document.update(overrides)
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def test_known_start_route_parses_first_waypoint_as_an_unscored_prior(tmp_path):
    module = load_module()
    route = write_route(tmp_path / "route.json")

    candidate = module.load_known_start(route, "map", "builtin")

    assert candidate.source == "known_start_route"
    assert candidate.score is None
    assert candidate.x == pytest.approx(5.15)
    assert candidate.y == pytest.approx(1.13)
    assert candidate.z == pytest.approx(-0.12)
    assert candidate.yaw_rad == pytest.approx(math.radians(14.2))


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"waypoints": [], "count": 0}, "empty_waypoints"),
        ({"frame": "camera_init"}, "frame_mismatch"),
        ({"body_frame_profile": "vn100"}, "body_frame_profile_mismatch"),
    ],
)
def test_known_start_route_rejects_unsafe_contracts(tmp_path, overrides, reason):
    module = load_module()
    route = write_route(tmp_path / "route.json", **overrides)

    with pytest.raises(module.KnownStartRouteError) as captured:
        module.load_known_start(route, "map", "builtin")

    assert captured.value.reason == reason


def test_known_start_prior_bypasses_coarse_score_while_global_fallback_does_not(
    tmp_path,
):
    module = load_module()
    prior = module.load_known_start(
        write_route(tmp_path / "route.json"), "map", "builtin"
    )
    low = module.InitializationCandidate(
        x=100.0,
        y=200.0,
        z=0.0,
        yaw_rad=0.0,
        score=0.10,
        source="global_search",
    )
    accepted = module.InitializationCandidate(
        x=101.0,
        y=201.0,
        z=0.0,
        yaw_rad=0.1,
        score=0.30,
        source="global_search",
    )

    attempts = module.initialization_attempts(prior, (low, accepted), 0.25, 4)

    assert attempts == (prior, accepted)


def test_global_fallback_rejects_nonbinary_pcd_with_typed_error(tmp_path):
    module = load_global_module()
    invalid = tmp_path / "invalid.pcd"
    invalid.write_bytes(b"not a point cloud")

    with pytest.raises(module.BinaryPcdError) as captured:
        module.load_pcd_xyz(invalid)

    assert captured.value.reason == "unsupported_header"


def test_handshake_rejects_stale_tracking_from_an_older_seed():
    module = load_module()
    stale = {"sequence": 10, "reset_count": 3, "message": "TRACKING"}

    assert not module.seed_was_acknowledged(stale, 10, 3)
    assert not module.seed_was_acknowledged(stale, 9, 3)
    assert not module.tracking_was_verified(stale, 10, 4, True)


def test_handshake_accepts_only_same_generation_after_verifying():
    module = load_module()
    manual = {"sequence": 11, "reset_count": 4, "message": "MANUAL_ALIGN"}
    tracking = {"sequence": 13, "reset_count": 4, "message": "TRACKING"}

    assert module.seed_was_acknowledged(manual, 10, 3)
    assert not module.tracking_was_verified(tracking, 11, 4, False)
    assert module.tracking_was_verified(tracking, 11, 4, True)


def test_stationary_stability_rejects_localization_walk_with_fixed_odometry():
    module = load_module()
    monitor = module.StationaryStabilityMonitor()

    assert monitor.observe((10.0, 4.0, 0.1), (0.0, 0.0, 0.0)) is None
    refusal = monitor.observe((10.3, 4.0, 0.1), (0.004, 0.0, 0.0))

    assert refusal == "LOCALIZATION_MOVED_WHILE_STATIONARY"


def test_stationary_stability_accepts_sensor_noise_inside_both_bounds():
    module = load_module()
    monitor = module.StationaryStabilityMonitor()

    assert monitor.observe((10.0, 4.0, 0.1), (0.0, 0.0, 0.0)) is None
    assert monitor.observe((10.03, 4.02, 0.11), (0.01, 0.0, 0.005)) is None


def test_known_start_rejects_a_route_that_faces_away_from_its_own_path(tmp_path):
    """The 0727 route stored -175 deg where the chair actually faced +4 deg:
    generated from a spin-in-place start, the polyline direction between
    consecutive samples was arbitrary. Seeded as the initialization prior it
    aimed ICP backwards, so verification could never converge and every
    startup fell through to the global search."""
    route = tmp_path / "reversed.json"
    route.write_text(json.dumps({
        "frame": "map",
        "body_frame_profile": "builtin",
        "count": 2,
        "waypoints": [
            {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": -175.4},
            {"x": 4.4, "y": 1.0, "z": 0.0, "yaw_deg": 10.0},
        ],
    }), encoding="utf-8")

    module = load_module()
    with pytest.raises(module.KnownStartRouteError) as raised:
        module.load_known_start(route, "map", "builtin")
    assert "start_heading_reversed" in str(raised.value)


def test_known_start_accepts_a_heading_that_merely_lags_the_first_turn(tmp_path):
    """Facing a little off the first segment is normal - the chair turns as
    it pulls away - so only a reversal is refused."""
    route = tmp_path / "ok.json"
    route.write_text(json.dumps({
        "frame": "map",
        "body_frame_profile": "builtin",
        "count": 2,
        "waypoints": [
            {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 3.7},
            {"x": 4.4, "y": 1.0, "z": 0.0, "yaw_deg": 10.0},
        ],
    }), encoding="utf-8")

    module = load_module()
    candidate = module.load_known_start(route, "map", "builtin")
    assert abs(math.degrees(candidate.yaw_rad) - 3.7) < 1e-6
