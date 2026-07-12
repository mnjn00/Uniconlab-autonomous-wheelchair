#!/usr/bin/env python3
"""Pure and static tests for the independent localization confidence boundary."""

import dataclasses
import hashlib
import importlib.util
import math
import random
from pathlib import Path
import sys
import struct
import threading
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts" / "localization_guard.py"
SPEC = importlib.util.spec_from_file_location("safety_localization_guard", str(SCRIPT))
guard = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = guard
SPEC.loader.exec_module(guard)

MAP_HASH = "1" * 64
POLICY_HASH = "2" * 64


def policy(**changes):
    values = dict(map_id="test-map", map_sha256=MAP_HASH, policy_sha256=POLICY_HASH)
    values.update(changes)
    return guard.LocalizationPolicy(**values)


def evidence(stamp=1.0, **changes):
    values = dict(
        stamp_s=stamp, source="untrusted_candidate", raw_state=2, reset_count=0,
        map_id="test-map", map_sha256=MAP_HASH, policy_sha256=POLICY_HASH,
        raw_score=0.9, position_std_m=0.08, yaw_std_rad=math.radians(2),
        scan_residual_m=0.05, inlier_ratio=0.9, innovation_nis=1.0,
        ambiguity_ratio=2.0, transform_age_s=0.02, odom_age_s=0.02,
        covariance_planar=(0.0064, 0, 0, 0, 0.0064, 0, 0, 0, math.radians(2) ** 2),
        linear_speed_mps=0.0, angular_speed_rps=0.0, stationary_duration_s=3.0,
        mission_canceled=True, initial_pose_fresh=True,
    )
    values.update(changes)
    return guard.CandidateEvidence(**values)


def initialize(core, start=1.0):
    result = None
    for index in range(20):
        stamp = start + index * 2.0 / 19.0
        result = core.evaluate(evidence(stamp), stamp)
    return result


def initialization_node(ros_now=10.0, monotonic_now=100.0):
    clock = [monotonic_now]
    node = guard.LocalizationGuardNode.__new__(guard.LocalizationGuardNode)
    node.policy = policy()
    node.core = guard.LocalizationGuardCore(node.policy)
    node.rospy = SimpleNamespace(
        Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: ros_now))
    )
    node.mission_canceled = True
    node.stationary_since = ros_now - node.policy.stationary_duration_s
    node.initial_pose = None
    node.initialization_attempt_pose = None
    node.initialization_attempt_deadline = None
    node.initialization_request_consumed = False
    node.initialization_attempt_timeout_s = 30.0
    node._monotonic = lambda: clock[0]
    node.last_odom_stamp = None
    node.odom = None
    return node, clock


def initial_pose_message(stamp=10.0):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id="map",
            stamp=SimpleNamespace(to_sec=lambda: stamp),
        )
    )


def test_process_construction_starts_with_unconsumed_initialization_request():
    node, _ = initialization_node()
    assert not node.initialization_request_consumed

def test_initialization_attempt_survives_bad_sample_then_clears_on_success():
    node, clock = initialization_node()
    node._initial_pose_callback(initial_pose_message())
    assert node.initialization_attempt_deadline == 130.0

    bad = node.core.evaluate(
        evidence(10.1, initial_pose_fresh=node._initial_pose_fresh(10.1),
                 transform_age_s=-0.01),
        10.1,
    )
    assert bad.state == guard.LOST
    assert bad.consecutive_good_samples == 0

    for index in range(30):
        stamp = 11.0 + index * 3.0 / 29.0
        result = node.core.evaluate(
            evidence(
                stamp, reset_count=1, relocalization_requested=True,
                relocalization_jump_evidence=True, position_jump_m=1.0,
                initial_pose_fresh=True,
            ),
            stamp,
        )
    assert clock[0] < node.initialization_attempt_deadline
    assert result.state == guard.OK
    node._clear_initialization_attempt_on_state_exit(result.state)
    assert node.initialization_attempt_deadline is None
    assert node.initialization_request_consumed


def test_duplicate_initial_pose_does_not_extend_initialization_attempt():
    node, clock = initialization_node()
    node._initial_pose_callback(initial_pose_message())
    first_deadline = node.initialization_attempt_deadline
    clock[0] += 10.0
    node._initial_pose_callback(initial_pose_message())
    assert node.initialization_attempt_deadline == first_deadline
    assert node.initialization_attempt_pose[1] == 100.0


def test_motion_invalidates_initialization_attempt():
    node, _ = initialization_node()
    node._initial_pose_callback(initial_pose_message())
    moving_odom = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 10.0)),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=node.policy.stationary_linear_mps),
                angular=SimpleNamespace(z=0.0),
            )
        ),
    )
    node._odom_callback(moving_odom)
    assert not node._initialization_attempt_active()
    assert node.initialization_attempt_deadline is None
    assert node.initialization_request_consumed
    node.stationary_since = 8.0
    node._initial_pose_callback(initial_pose_message())
    assert node.initialization_attempt_deadline is None


def test_initialization_attempt_deadline_is_monotonic_and_fail_closed_at_boundary():
    node, clock = initialization_node()
    node._initial_pose_callback(initial_pose_message())
    deadline = node.initialization_attempt_deadline
    clock[0] = deadline - 0.001
    assert node._initialization_attempt_active()
    clock[0] = deadline
    assert not node._initialization_attempt_active()
    assert node.initialization_attempt_deadline is None
    node._initial_pose_callback(initial_pose_message())
    assert node.initialization_attempt_deadline is None
    assert node.initialization_request_consumed
    clock[0] = deadline + 0.001
    assert not node._initialization_attempt_active()


def test_relocalization_initial_pose_freshness_remains_ros_time_based():
    node, clock = initialization_node()
    node.core.state = guard.LOST
    node._initial_pose_callback(initial_pose_message())
    clock[0] += 1000.0
    assert node._initial_pose_fresh(10.0)
    assert not node._initial_pose_fresh(
        10.0 + node.policy.relocalization_span_s + node.policy.max_candidate_age_s + 0.001
    )


def test_map_loader_verifies_yaml_and_image_hashes(tmp_path):
    image = b"P2\n3 2\n255\n255 0 255\n255 255 255\n"
    (tmp_path / "map.pgm").write_bytes(image)
    metadata = b"image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\n"
    path = tmp_path / "map.yaml"
    path.write_bytes(metadata)
    loaded = guard.load_occupancy_map(
        str(path), hashlib.sha256(metadata).hexdigest(), hashlib.sha256(image).hexdigest()
    )
    assert loaded.width == 3 and loaded.height == 2
    assert loaded.occupied[0] == pytest.approx((0.15, 0.15))
    with pytest.raises(ValueError, match="hash mismatch"):
        guard.load_occupancy_map(str(path), "0" * 64)
    with pytest.raises(ValueError, match="image hash mismatch"):
        guard.load_occupancy_map(str(path), hashlib.sha256(metadata).hexdigest(), "0" * 64)
def test_cloud_sampling_is_bounded_deterministic_and_spatially_distributed():
    organized = guard.uniform_sample_uvs(1000, 2)
    unorganized = guard.uniform_sample_uvs(2048, 1)
    assert len(organized) == len(unorganized) == 1024
    assert organized == guard.uniform_sample_uvs(1000, 2)
    assert organized[0] == (0, 0) and organized[-1] == (999, 1)
    assert unorganized[0] == (0, 0) and unorganized[-1] == (2047, 0)
    assert guard.uniform_sample_uvs(1, 1) == ((0, 0),)
    for invalid in ((0, 1, 16), (1, 0, 16), (1, 1, 0), (1, 1, 1025)):
        with pytest.raises(ValueError, match="sample bound"):
            guard.uniform_sample_uvs(*invalid)


def test_scan_selection_applies_full_extrinsic_and_rejects_non_geometry():
    selection_policy = policy()
    half = math.sqrt(0.5)
    # A +90 degree roll maps sensor y to base z and sensor z to -base y.
    walls = [(2.0, 0.5, 0.0)] * 16
    selected = guard.select_scan_points(
        walls, (0.0, 0.0, 0.2), (half, 0.0, 0.0, half), selection_policy
    )
    assert len(selected) == 16
    assert selected[0] == pytest.approx((2.0, 0.0))
    rejected = (
        [(0.49, 0.0, 0.5)] * 16
        + [(20.01, 0.0, 0.5)] * 16
        + [(2.0, 0.0, 0.14)] * 16
    )
    with pytest.raises(ValueError, match="insufficient"):
        guard.select_scan_points(
            rejected, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), selection_policy
        )
    with pytest.raises(ValueError, match="non-finite"):
        guard.select_scan_points(
            [(math.nan, 0.0, 1.0)] * 16,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), selection_policy,
        )
    with pytest.raises(ValueError, match="quaternion"):
        guard.select_scan_points(
            [(2.0, 0.0, 1.0)] * 16,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), selection_policy,
        )


def test_scan_selection_downsamples_after_filtering_and_median_is_robust():
    selected = guard.select_scan_points(
        [(2.0 + index / 1000.0, 0.0, 1.0) for index in range(256)],
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), policy(),
    )
    assert len(selected) == 128
    assert selected == guard.select_scan_points(
        [(2.0 + index / 1000.0, 0.0, 1.0) for index in range(256)],
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), policy(),
    )
    map_points = tuple((float(index), 0.0) for index in range(16))
    occupancy = guard.OccupancyMap(0.05, 0, 0, 0, 100, 100, map_points, MAP_HASH, MAP_HASH)
    scan = map_points + ((100.0, 100.0),)
    metrics = guard.compute_scan_metrics(
        occupancy, scan, guard.Pose2D(0, 0, 0), 0.20, 0.65
    )
    assert metrics.residual_m == pytest.approx(0.0)



def test_indexed_nearest_distance_matches_bounded_brute_force():
    def make_map(cells, yaw=0.0):
        template = guard.OccupancyMap(
            0.1, -1.25, 2.5, yaw, 24, 19, (), MAP_HASH, MAP_HASH,
            frozenset(cells),
        )
        occupied = tuple(template._cell_center(row, column) for row, column in cells)
        return dataclasses.replace(template, occupied=occupied)

    def brute_force(occupancy, x, y):
        cosine, sine = math.cos(occupancy.origin_yaw), math.sin(occupancy.origin_yaw)
        relative_x, relative_y = x - occupancy.origin_x, y - occupancy.origin_y
        column = int(math.floor(
            (cosine * relative_x + sine * relative_y) / occupancy.resolution
        ))
        map_y = (-sine * relative_x + cosine * relative_y) / occupancy.resolution
        row = occupancy.height - 1 - int(math.floor(map_y))
        maximum_cells = max(1, int(math.ceil(1.0 / occupancy.resolution)))
        distances = [
            math.hypot(x - ox, y - oy)
            for candidate_row, candidate_column in occupancy.occupied_cells
            if abs(candidate_row - row) <= maximum_cells
            and abs(candidate_column - column) <= maximum_cells
            for ox, oy in [occupancy._cell_center(candidate_row, candidate_column)]
        ]
        return min(distances) if distances else 1.0 + occupancy.resolution

    rng = random.Random(903)
    cells = {
        (row, column)
        for row in range(19)
        for column in range(24)
        if rng.random() < 0.12
    }
    cells.update({(0, 0), (18, 23), (9, 12)})
    for yaw in (0.0, math.radians(31.0), -math.radians(47.0)):
        occupancy = make_map(cells, yaw)
        queries = [
            occupancy._cell_center(0, 0),
            occupancy._cell_center(18, 23),
            occupancy._cell_center(9, 12),
            (occupancy.origin_x - 2.0, occupancy.origin_y - 2.0),
        ]
        queries.extend(
            (rng.uniform(-2.0, 2.5), rng.uniform(1.0, 5.5))
            for _ in range(100)
        )
        for x, y in queries:
            assert occupancy.nearest_distance(x, y) == pytest.approx(
                brute_force(occupancy, x, y)
            )


def test_occupancy_row_index_is_immutable_and_empty_map_is_compatible():
    occupancy = guard.OccupancyMap(
        0.1, 0.0, 0.0, 0.0, 4, 3, ((0.15, 0.15),),
        MAP_HASH, MAP_HASH, frozenset({(1, 1)}),
    )
    assert occupancy.occupied_columns_by_row == ((), (1,), ())
    with pytest.raises(dataclasses.FrozenInstanceError):
        occupancy.occupied_columns_by_row = ()
    with pytest.raises(TypeError):
        occupancy.occupied_columns_by_row[1][0] = 2

    empty = guard.OccupancyMap(
        0.1, 0.0, 0.0, 0.0, 4, 3, (), MAP_HASH, MAP_HASH
    )
    assert empty.occupied_columns_by_row == ((), (), ())
    assert math.isinf(empty.nearest_distance(0.0, 0.0))


def test_scan_metrics_reject_one_metre_and_twenty_degree_plausible_wrong_pose():
    points = ((0.0, 0.0), (1.0, 0.0), (0.1, 1.3), (1.7, 0.4))
    occupancy = guard.OccupancyMap(
        0.05, 0, 0, 0, 100, 100, points, MAP_HASH, MAP_HASH
    )
    correct = guard.compute_scan_metrics(
        occupancy, points, guard.Pose2D(0, 0, 0), 0.20, 0.65
    )
    shifted = guard.compute_scan_metrics(
        occupancy, points, guard.Pose2D(1, 0, 0), 0.20, 0.65
    )
    rotated = guard.compute_scan_metrics(
        occupancy, points, guard.Pose2D(0, 0, math.radians(20)), 0.20, 0.65
    )
    assert correct.residual_m == pytest.approx(0.0)
    assert correct.inlier_ratio == 1.0 and correct.ambiguity_ratio >= 1.5
    assert shifted.residual_m > 0.20 or shifted.ambiguity_ratio < 1.5
    assert rotated.residual_m > 0.20 or rotated.ambiguity_ratio < 1.5


def test_scan_ambiguity_ignores_lower_median_alternative_below_inlier_floor():
    class ProfileMap:
        resolution = 0.05

        @staticmethod
        def nearest_distance(x, y):
            if abs(y) > 1e-9:
                return 1.0
            index = round(x / 10.0)
            if abs(x - index * 10.0) < 1e-9:
                return 0.065
            alias_index = round((x - 1.0) / 10.0)
            if abs(x - (alias_index * 10.0 + 1.0)) < 1e-9:
                return 0.05 if alias_index < 64 else 0.30
            return 1.0

    points = tuple((index * 10.0, 0.0) for index in range(100))
    metrics = guard.compute_scan_metrics(
        ProfileMap(), points, guard.Pose2D(0, 0, 0), 0.20, 0.65
    )
    assert metrics.residual_m == pytest.approx(0.065)
    assert metrics.inlier_ratio == 1.0
    assert math.isfinite(metrics.ambiguity_ratio)
    assert metrics.ambiguity_ratio >= 1.50
    assert metrics.ambiguity_ratio <= 1_000_000.0
    assert struct.pack("f", metrics.ambiguity_ratio)


def test_hypothesis_score_is_contract_normalized_and_scale_invariant():
    score = guard._hypothesis_score(0.10, 0.825, 0.20, 0.65)
    scaled = guard._hypothesis_score(0.25, 0.825, 0.50, 0.65)
    assert score == pytest.approx(1.0)
    assert scaled == pytest.approx(score)

    observed_candidate = guard._hypothesis_score(0.05816, 0.96094, 0.20, 0.65)
    observed_alias = guard._hypothesis_score(0.04661, 0.85938, 0.20, 0.65)
    assert observed_alias / observed_candidate == pytest.approx(1.58, abs=0.01)


def test_hypothesis_score_ordering_and_threshold_boundary():
    candidate = guard._hypothesis_score(0.10, 0.90, 0.20, 0.65)
    same_quality_alias = guard._hypothesis_score(0.10, 0.90, 0.20, 0.65)
    better_alias = guard._hypothesis_score(0.09, 0.90, 0.20, 0.65)
    boundary_alias = guard._hypothesis_score(0.20, 0.65, 0.20, 0.65)
    assert same_quality_alias <= candidate
    assert better_alias < candidate
    assert boundary_alias == pytest.approx(2.0)


def test_threshold_boundary_alternative_is_viable():
    class BoundaryAliasMap:
        resolution = 0.05

        @staticmethod
        def nearest_distance(x, y):
            if abs(y) > 1e-9:
                return 1.0
            index = round(x / 10.0)
            if abs(x - index * 10.0) < 1e-9:
                return 0.05
            alias_index = round((x - 1.0) / 10.0)
            if abs(x - (alias_index * 10.0 + 1.0)) < 1e-9:
                return 0.20 if alias_index < 13 else 1.0
            return 1.0

    points = tuple((index * 10.0, 0.0) for index in range(20))
    metrics = guard.compute_scan_metrics(
        BoundaryAliasMap(), points, guard.Pose2D(0, 0, 0), 0.20, 0.65
    )
    assert metrics.ambiguity_ratio == pytest.approx(8.0)


def test_perfect_unique_hypothesis_uses_float32_safe_cap():
    points = ((0.3, 0.7), (1.4, 0.2), (0.8, 1.9), (2.1, 1.1))
    occupancy = guard.OccupancyMap(
        0.05, 0.0, 0.0, 0.0, 100, 100, points, MAP_HASH, MAP_HASH
    )
    metrics = guard.compute_scan_metrics(
        occupancy, points, guard.Pose2D(0, 0, 0), 0.20, 0.65
    )
    assert metrics.residual_m == 0.0
    assert metrics.inlier_ratio == 1.0
    assert metrics.ambiguity_ratio == 1_000_000.0
    assert struct.pack("f", metrics.ambiguity_ratio)

def test_scan_ambiguity_combines_observed_residual_and_inlier_evidence():
    class ObservedProfileMap:
        resolution = 0.05

        @staticmethod
        def nearest_distance(x, y):
            if abs(y) > 1e-9:
                return 1.0
            index = round(x / 10.0)
            if abs(x - index * 10.0) < 1e-9:
                return 0.05290 if index < 558 else 0.30
            alias_index = round((x + 1.0) / 10.0)
            if abs(x - (alias_index * 10.0 - 1.0)) < 1e-9:
                return 0.05176 if alias_index < 391 else 0.30
            return 1.0

    points = tuple((index * 10.0, 0.0) for index in range(576))
    metrics = guard.compute_scan_metrics(
        ObservedProfileMap(), points, guard.Pose2D(0, 0, 0), 0.20, 0.65
    )
    assert metrics.residual_m == pytest.approx(0.05290)
    assert metrics.inlier_ratio == pytest.approx(0.96875)
    assert metrics.ambiguity_ratio == pytest.approx(3.33, abs=0.02)
    assert struct.pack("f", metrics.ambiguity_ratio)


def test_scan_ambiguity_rejects_alternative_with_strictly_better_score():
    class BetterAliasMap:
        resolution = 0.05

        @staticmethod
        def nearest_distance(x, y):
            if abs(y) > 1e-9:
                return 1.0
            index = round(x / 10.0)
            if abs(x - index * 10.0) < 1e-9:
                return 0.06
            alias_index = round((x - 1.0) / 10.0)
            if abs(x - (alias_index * 10.0 + 1.0)) < 1e-9:
                return 0.05
            return 1.0

    points = tuple((index * 10.0, 0.0) for index in range(20))
    metrics = guard.compute_scan_metrics(
        BetterAliasMap(), points, guard.Pose2D(0, 0, 0), 0.20, 0.65
    )
    assert metrics.residual_m == pytest.approx(0.06)
    assert metrics.inlier_ratio == 1.0
    assert metrics.ambiguity_ratio < 1.50


def test_scan_ambiguity_retains_viable_one_metre_alias():
    points = tuple((index * 0.1, 0.0) for index in range(10))
    occupied = points + tuple((x + 1.0, y) for x, y in points)
    occupancy = guard.OccupancyMap(
        0.05, 0, 0, 0, 100, 100, occupied, MAP_HASH, MAP_HASH
    )
    metrics = guard.compute_scan_metrics(
        occupancy, points, guard.Pose2D(0, 0, 0), 0.20, 0.65
    )
    assert metrics.residual_m == pytest.approx(0.0)
    assert metrics.inlier_ratio == 1.0
    assert metrics.ambiguity_ratio < 1.50
    result = guard.LocalizationGuardCore(policy()).evaluate(
        evidence(
            scan_residual_m=metrics.residual_m,
            inlier_ratio=metrics.inlier_ratio,
            ambiguity_ratio=metrics.ambiguity_ratio,
        ),
        1.0,
    )
    assert result.state == guard.LOST
    assert result.safety_state == guard.SAFETY_STOP
    assert result.reason_mask & guard.REASON_LOCALIZATION_INCONSISTENT
    assert not result.reason_mask & guard.REASON_RESET_REJECTED


def test_poor_scan_candidate_fails_independently_of_unambiguous_ratio():
    class PoorMap:
        resolution = 0.05

        @staticmethod
        def nearest_distance(_x, _y):
            return 0.30

    metrics = guard.compute_scan_metrics(
        PoorMap(), ((0.0, 0.0),) * 20, guard.Pose2D(0, 0, 0), 0.20, 0.65
    )
    assert metrics.residual_m == pytest.approx(0.30)
    assert metrics.inlier_ratio == 0.0
    assert math.isfinite(metrics.ambiguity_ratio)
    result = guard.LocalizationGuardCore(policy()).evaluate(
        evidence(
            scan_residual_m=metrics.residual_m,
            inlier_ratio=metrics.inlier_ratio,
            ambiguity_ratio=metrics.ambiguity_ratio,
        ),
        1.0,
    )
    assert result.state == guard.LOST
    assert result.safety_state == guard.SAFETY_STOP
    assert result.reason_mask & guard.REASON_LOCALIZATION_INCONSISTENT
    assert not result.reason_mask & guard.REASON_RESET_REJECTED


def test_covariance_nis_continuity_and_frozen_pose_are_independently_derived():
    covariance = (0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01)
    assert guard.covariance_nis(0.1, 0, 0, covariance) == pytest.approx(1.0)
    assert math.isinf(guard.covariance_nis(0, 0, 0, (0,) * 9))
    tracker = guard.IndependentEvidenceTracker()
    tracker.derive(guard.Pose2D(0, 0, 0), guard.Pose2D(0, 0, 0), covariance)
    jump, yaw_jump, odom_delta, nis = tracker.derive(
        guard.Pose2D(0, 0, 0), guard.Pose2D(0.5, 0, 0), covariance
    )
    assert jump == yaw_jump == 0.0 and odom_delta == 0.5 and nis > 12.84
    result = guard.LocalizationGuardCore(policy()).evaluate(
        evidence(odom_delta_m=odom_delta, position_jump_m=jump, innovation_nis=nis), 1.0
    )
    assert result.state == guard.LOST
    assert result.safety_state == guard.SAFETY_STOP
    assert result.reason_mask & guard.REASON_LOCALIZATION_INCONSISTENT
    assert not result.reason_mask & guard.REASON_RESET_REJECTED
def test_reset_change_and_source_stamp_regression_latch_loss():
    core = guard.LocalizationGuardCore(policy())
    assert initialize(core).state == guard.OK
    reset = core.evaluate(evidence(3.2, reset_count=1), 3.2)
    assert reset.state == guard.LOST
    assert reset.reason_mask & guard.REASON_RESET_REJECTED

    core = guard.LocalizationGuardCore(policy())
    assert initialize(core, 10.0).state == guard.OK
    assert core.evaluate(evidence(9.0), 10.0).state == guard.LOST
    regressed = core.evaluate(evidence(9.1), 10.0)
    assert regressed.state == guard.LOST
    assert regressed.reason_mask & guard.REASON_CLOCK


def test_map_transform_rotates_odom_pose_and_covariance_before_nis():
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=10.0, y=-2.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sqrt(0.5), w=math.sqrt(0.5)),
    ))
    pose = guard.transform_planar_pose(transform, guard.Pose2D(2.0, 0.0, 0.0))
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((10.0, 0.0, math.pi / 2.0))
    covariance = guard.rotate_planar_covariance(
        (1.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 9.0), math.pi / 2.0
    )
    assert covariance == pytest.approx((4.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 9.0))



@pytest.mark.parametrize("changes, reason", [
    ({"covariance_planar": (0,) * 9}, guard.REASON_CORRUPT_DATA),
    ({"transform_age_s": 0.251}, guard.REASON_TF),
    ({"tf_authority_count": 0}, guard.REASON_TF),
    ({"tf_authority_count": 2}, guard.REASON_TF),
    ({"stamp_s": 0.0}, guard.REASON_SENSOR_STALE),
    ({"map_sha256": "3" * 64}, guard.REASON_MAP_MISMATCH),
    ({"policy_sha256": "3" * 64}, guard.REASON_POLICY_MISMATCH),
    ({"scan_residual_m": 0.201}, guard.REASON_LOCALIZATION_INCONSISTENT),
    ({"inlier_ratio": 0.649}, guard.REASON_LOCALIZATION_INCONSISTENT),
    ({"ambiguity_ratio": 1.49}, guard.REASON_LOCALIZATION_INCONSISTENT),
])
def test_missing_stale_duplicate_or_inconsistent_evidence_is_lost_stop(changes, reason):
    core = guard.LocalizationGuardCore(policy())
    initialize(core)
    candidate = evidence(**changes) if "stamp_s" in changes else evidence(3.2, **changes)
    result = core.evaluate(candidate, 3.2)
    assert result.state == guard.LOST and result.safety_state == guard.SAFETY_STOP
    assert result.reason_mask & reason


def test_startup_loss_adopts_baseline_then_requires_later_reset_increment():
    core = guard.LocalizationGuardCore(policy())
    core.force_loss()
    assert core.evaluate(evidence(1.0), 1.0).state == guard.LOST
    assert core._loss_reset_count == 0
    for index in range(30):
        stamp = 2.0 + index * 3.0 / 29.0
        result = core.evaluate(evidence(
            stamp, reset_count=1, relocalization_requested=True,
            relocalization_jump_evidence=True, position_jump_m=1.0,
        ), stamp)
        if index < 29:
            assert result.safety_state == guard.SAFETY_STOP
    assert result.state == guard.OK
    assert result.consecutive_good_samples == 30
    assert result.safety_state == guard.SAFETY_CLEAR
def test_explicit_recovery_opens_a_new_input_chronology_epoch():
    class Message:
        def __init__(self, stamp):
            self.header = SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp))

    class Publisher:
        def publish(self, _message):
            pass

    node = guard.LocalizationGuardNode.__new__(guard.LocalizationGuardNode)
    node.policy = policy()
    node.core = guard.LocalizationGuardCore(node.policy)
    node.rospy = SimpleNamespace(
        Time=SimpleNamespace(
            now=lambda: SimpleNamespace(to_sec=lambda: 11.0),
            from_sec=lambda value: value,
        )
    )
    node.LocalizationStatus = lambda: SimpleNamespace(header=SimpleNamespace())
    node.SafetySignal = lambda: SimpleNamespace(header=SimpleNamespace())
    node.status_pub = Publisher()
    node.signal_pub = Publisher()
    node.last_candidate_source_stamp = None
    node.output_sequence = 0
    startup = node.core.evaluate(evidence(1.0), 1.0)
    assert startup.state == guard.INITIALIZING
    node.last_cloud_stamp = 10.0
    node.last_odom_stamp = 10.0

    node._cloud_callback(Message(10.0))
    node._cloud_callback(Message(9.0))
    node._cloud_callback(Message(9.1))
    assert node.last_cloud_stamp == 10.0
    assert node.core.state == guard.LOST

    node.core.adopt_loss_reset_baseline(0)
    assert node.core.consume_recovery_epoch(1, requested=True)
    node._begin_input_epoch()
    node._cloud_callback(Message(9.1))
    assert node.last_cloud_stamp == 9.1


def test_bad_sample_after_ok_latches_loss_and_requires_relocalization_reset():
    core = guard.LocalizationGuardCore(policy())
    assert initialize(core).state == guard.OK
    failed = core.evaluate(evidence(3.2, transform_age_s=-0.01), 3.2)
    assert failed.state == guard.LOST
    assert failed.reason_mask & guard.REASON_TF
    assert failed.safety_state == guard.SAFETY_STOP

    rejected = core.evaluate(evidence(3.3), 3.3)
    assert rejected.state == guard.LOST
    assert rejected.reason_mask & guard.REASON_RESET_REJECTED
    assert rejected.safety_state == guard.SAFETY_STOP

def test_initialization_and_explicit_relocalization_windows_never_clear_early():
    core = guard.LocalizationGuardCore(policy())
    result = initialize(core)
    assert result.state == guard.OK and result.consecutive_good_samples == 20
    assert result.safety_state == guard.SAFETY_CLEAR
    assert core.evaluate(evidence(3.2, scan_residual_m=1.0), 3.2).state == guard.LOST
    for index in range(30):
        stamp = 4.0 + index * 3.0 / 29.0
        result = core.evaluate(evidence(
            stamp, reset_count=1, relocalization_requested=True,
            relocalization_jump_evidence=True, position_jump_m=1.0,
        ), stamp)
        if index < 29:
            assert result.state == guard.RELOCALIZING
            assert result.safety_state == guard.SAFETY_STOP
    assert result.state == guard.OK and result.safety_state == guard.SAFETY_CLEAR


def test_non_raw_ok_and_unqualified_policy_never_clear():
    non_raw_ok = guard.LocalizationGuardCore(policy()).evaluate(evidence(raw_state=3), 1.0)
    assert non_raw_ok.state == guard.LOST and non_raw_ok.safety_state == guard.SAFETY_STOP
    unqualified = guard.LocalizationGuardCore(policy(calibration_qualified=False)).evaluate(evidence(), 1.0)
    assert unqualified.state == guard.LOST
    assert unqualified.safety_state == guard.SAFETY_STOP
    assert unqualified.reason_mask & guard.REASON_POLICY_MISMATCH


def test_only_explicit_synthetic_qualification_is_clear_capable():
    navigation_config = ROOT.parent / "wheelchair_navigation" / "config"
    assert guard.has_explicit_synthetic_qualification(
        str(navigation_config / "localization_confidence_sim.yaml")
    )
    assert not guard.has_explicit_synthetic_qualification(
        str(navigation_config / "localization_confidence.yaml")
    )


def test_policy_parser_requires_exact_frozen_scan_selection(tmp_path):
    import yaml

    source = ROOT.parent / "wheelchair_navigation" / "config" / "localization_confidence.yaml"
    document = yaml.safe_load(source.read_text())
    exact_path = tmp_path / "exact.yaml"
    exact_path.write_text(yaml.safe_dump(document))
    loaded = guard.load_localization_policy(str(exact_path))
    assert (
        loaded.scan_candidate_samples,
        loaded.scan_minimum_selected_points,
        loaded.scan_maximum_selected_points,
        loaded.scan_planar_range_min_m,
        loaded.scan_planar_range_max_m,
        loaded.scan_base_z_min_m,
        loaded.scan_base_z_max_m,
    ) == (1024, 16, 128, 0.5, 20.0, 0.15, 2.0)

    for key, value in (
            ("candidate_samples", None),
            ("minimum_selected_points", 15),
            ("maximum_selected_points", 129),
            ("planar_range_min_m", 0.49),
            ("planar_range_max_m", 20.01),
            ("base_z_min_m", 0.14),
            ("base_z_max_m", 2.01)):
        invalid = yaml.safe_load(source.read_text())
        if value is None:
            del invalid["thresholds"]["scan_selection"][key]
        else:
            invalid["thresholds"]["scan_selection"][key] = value
        path = tmp_path / ("invalid-%s.yaml" % key)
        path.write_text(yaml.safe_dump(invalid))
        with pytest.raises((TypeError, ValueError), match="scan selection"):
            guard.load_localization_policy(str(path))




def test_cold_start_stop_preserves_startup_state_and_output_generations():
    class Message:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    node = guard.LocalizationGuardNode.__new__(guard.LocalizationGuardNode)
    node.LocalizationStatus = Message
    node.SafetySignal = Message
    node.rospy = SimpleNamespace(
        Time=SimpleNamespace(from_sec=lambda value: value)
    )
    node.policy = policy()
    node.core = guard.LocalizationGuardCore(node.policy)
    node.status_pub = Publisher()
    node.signal_pub = Publisher()
    node.output_sequence = 0

    node._publish_stop(0.5, guard.REASON_SENSOR_STALE)
    assert node.core.state == guard.UNINITIALIZED
    assert node.status_pub.messages[-1].state == guard.UNINITIALIZED
    assert node.status_pub.messages[-1].reason_mask & guard.REASON_SENSOR_STALE
    assert node.signal_pub.messages[-1].state == guard.SAFETY_STOP

    first = node.core.evaluate(evidence(1.0), 1.0)
    node._publish_result(first, 1.0)
    node._publish_stop(1.3, guard.REASON_SENSOR_STALE)
    node._publish_stop(1.35, guard.REASON_SENSOR_STALE)

    assert node.core.state == guard.LOST
    assert [message.sequence for message in node.status_pub.messages] == [1, 2, 3, 4]
    assert [message.sequence for message in node.signal_pub.messages] == [1, 2, 3, 4]
    for status, signal in zip(node.status_pub.messages, node.signal_pub.messages):
        assert signal.header is not status.header
        assert signal.header.stamp == status.evaluation_stamp
        assert signal.sequence == status.sequence
    assert [message.evaluation_stamp for message in node.status_pub.messages] == [
        0.5, first.evaluation_stamp_s, 1.3, 1.35
    ]
    assert [message.state for message in node.status_pub.messages] == [
        guard.UNINITIALIZED, first.state, guard.LOST, guard.LOST
    ]
    assert all(message.state == guard.SAFETY_STOP for message in node.signal_pub.messages)


def test_zero_duplicate_and_regressing_candidate_sequences_are_rejected():
    sequences = guard.CandidateSequenceTracker()
    assert not sequences.observe(0)
    assert sequences.observe(3)
    assert not sequences.observe(3)
    assert not sequences.observe(2)
    assert sequences.watchdog_sequence() == 2
    assert sequences.observe(4)

def test_startup_waits_for_expected_tf_authority_after_candidate_stamped_lookup():
    node = guard.LocalizationGuardNode.__new__(guard.LocalizationGuardNode)
    node.tf_authorities = {}
    node._monotonic = lambda: 10.0

    transform = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"), child_frame_id="odom"
    )
    message = SimpleNamespace(
        _connection_header={"callerid": "/localization_adapter"},
        transforms=(transform,),
    )

    class DelayedAuthorityCondition:
        def __enter__(self):
            return self

        def __exit__(self, _type, _value, _traceback):
            return False

        def wait(self, _timeout):
            node._tf_callback(message)

        def notify_all(self):
            pass

    node.tf_authority_condition = DelayedAuthorityCondition()
    authority_count = node._await_single_tf_authority()

    result = guard.LocalizationGuardCore(policy()).evaluate(
        evidence(tf_authority_count=authority_count), 1.0
    )
    assert authority_count == 1
    assert not result.reason_mask & guard.REASON_TF
    assert result.state == guard.INITIALIZING


def test_rogue_only_map_to_odom_authority_remains_lost():
    node = guard.LocalizationGuardNode.__new__(guard.LocalizationGuardNode)
    node.tf_authorities = {}
    node.tf_authority_condition = threading.Condition()
    node._tf_callback(SimpleNamespace(
        _connection_header={"callerid": "/rogue_localizer"},
        transforms=(SimpleNamespace(
            header=SimpleNamespace(frame_id="map"), child_frame_id="odom"
        ),),
    ))

    result = guard.LocalizationGuardCore(policy()).evaluate(
        evidence(tf_authority_count=node._active_tf_authorities(1.0)), 1.0
    )
    assert result.state == guard.LOST
    assert result.reason_mask & guard.REASON_TF


def test_expected_plus_rogue_map_to_odom_authorities_remain_lost():
    node = guard.LocalizationGuardNode.__new__(guard.LocalizationGuardNode)
    node.tf_authorities = {}
    node.tf_authority_condition = threading.Condition()
    transform = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"), child_frame_id="odom"
    )
    for caller in ("/localization_adapter", "/rogue_localizer"):
        node._tf_callback(SimpleNamespace(
            _connection_header={"callerid": caller}, transforms=(transform,)
        ))

    result = guard.LocalizationGuardCore(policy()).evaluate(
        evidence(tf_authority_count=node._active_tf_authorities(1.0)), 1.0
    )
    assert result.state == guard.LOST
    assert result.reason_mask & guard.REASON_TF


def test_wrong_frame_expected_authority_remains_lost():
    node = guard.LocalizationGuardNode.__new__(guard.LocalizationGuardNode)
    node.tf_authorities = {}
    node.tf_authority_condition = threading.Condition()
    node._tf_callback(SimpleNamespace(
        _connection_header={"callerid": "/localization_adapter"},
        transforms=(SimpleNamespace(
            header=SimpleNamespace(frame_id="map"), child_frame_id="base_link"
        ),),
    ))

    result = guard.LocalizationGuardCore(policy()).evaluate(
        evidence(tf_authority_count=node._active_tf_authorities(1.0)), 1.0
    )
    assert result.state == guard.LOST
    assert result.reason_mask & guard.REASON_TF


def test_stale_tf_remains_lost_after_startup_authority_wait():
    result = guard.LocalizationGuardCore(policy()).evaluate(
        evidence(transform_age_s=0.251, tf_authority_count=1), 1.0
    )
    assert result.state == guard.LOST
    assert result.reason_mask & guard.REASON_TF

def test_map_to_odom_lookup_is_candidate_stamped_not_latest():
    source = SCRIPT.read_text()
    derive_evidence = source.split("def _derive_evidence", 1)[1].split(
        "def _publish_result", 1
    )[0]
    assert 'self._lookup("map", "odom", stamp)' in derive_evidence
    assert 'self._lookup("map", "odom", self.rospy.Time(0))' not in derive_evidence
    assert 'odom.child_frame_id != "base_footprint"' in derive_evidence
    assert 'self._lookup("base_link", cloud.header.frame_id, cloud.header.stamp)' in derive_evidence
    assert 'self._lookup("base_footprint", cloud.header.frame_id, cloud.header.stamp)' not in derive_evidence

    synchronized = guard.LocalizationGuardCore(policy()).evaluate(
        evidence(transform_age_s=0.011), 1.0
    )
    assert not synchronized.reason_mask & guard.REASON_TF

def test_ros_adapter_is_lazy_queue_one_confidence_only_and_launch_has_single_node():
    source = SCRIPT.read_text()
    launch = (ROOT / "launch" / "safety.launch").read_text()
    assert "import rospy" not in source.split("class LocalizationGuardNode", 1)[0]
    assert source.count("rospy.Publisher(self.STATUS_TOPIC") == 1
    assert source.count("rospy.Publisher(self.SIGNAL_TOPIC") == 1
    assert "queue_size=1" in source
    assert "TransformBroadcaster" not in source and "sendTransform" not in source
    assert "cmd_vel" not in source and "Twist" not in source
    for other_script in ROOT.parent.glob("*/scripts/*.py"):
        if other_script != SCRIPT:
            other_source = other_script.read_text()
            assert 'Publisher("/localization/status"' not in other_source
            assert 'Publisher("/safety/localization"' not in other_source
    assert launch.count('type="localization_guard.py"') == 1
    for topic in ("/localization/candidate", "/sensors/lidar/points", "/odom",
                  "/localization/status", "/safety/localization"):
        assert topic in source or topic in launch


def test_reference_core_contract_remains_frozen_and_fail_closed():
    assert dataclasses.is_dataclass(guard.LocalizationPolicy)
    assert guard.LocalizationPolicy.__dataclass_params__.frozen
    assert dataclasses.is_dataclass(guard.CandidateEvidence)
    assert guard.CandidateEvidence.__dataclass_params__.frozen
    assert guard.LocalizationGuardCore(policy()).evaluate(
        evidence(raw_state=guard.LOST), 1.0
    ).safety_state == guard.SAFETY_STOP
    with pytest.raises(dataclasses.FrozenInstanceError):
        policy().max_candidate_age_s = 1.0
