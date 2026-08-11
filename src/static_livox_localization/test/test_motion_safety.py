import math
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPT_DIR = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
try:
    from cloud_points import (COLLISION_MAX_HEIGHT_M,
                              COLLISION_MIN_HEIGHT_M)
    from motion_safety import (PoseMotionEstimator, filter_obstacle_points,
                               motion_hold_reason, stopping_envelope,
                               swept_footprint_collision)
finally:
    sys.path.remove(str(SCRIPT_DIR))


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def update(estimator, stamp, receipt, x, y, yaw=0.0,
           frame="camera_init", child="body"):
    return estimator.update(
        source_stamp_s=stamp,
        receipt_stamp_s=receipt,
        frame_id=frame,
        child_frame_id=child,
        x=x,
        y=y,
        quaternion_xyzw=quaternion_from_yaw(yaw))


def test_pose_deltas_supply_real_speed_when_odometry_twist_is_unusable():
    estimator = PoseMotionEstimator("camera_init", "body")
    first = update(estimator, 10.0, 10.01, 0.0, 0.0)
    second = update(estimator, 10.1, 10.11, 0.06, 0.0, yaw=0.04)

    assert not first.valid
    assert second.valid
    assert second.linear_speed_mps == pytest.approx(0.6)
    assert second.angular_speed_rps == pytest.approx(0.4)
    assert motion_hold_reason(second, now_s=10.2, max_age_s=0.25) == ""


def test_motion_estimate_fails_closed_on_frame_mismatch_and_time_gap():
    estimator = PoseMotionEstimator("camera_init", "body", max_dt_s=0.3)
    wrong_frame = update(
        estimator, 10.0, 10.0, 0.0, 0.0, frame="map")
    assert wrong_frame.reason == "ODOM_FRAME"

    assert not update(estimator, 11.0, 11.0, 0.0, 0.0).valid
    gap = update(estimator, 11.5, 11.5, 0.1, 0.0)
    assert not gap.valid
    assert gap.reason == "ODOM_GAP"
    assert motion_hold_reason(gap, now_s=11.5, max_age_s=0.25) == "ODOM_GAP"


def test_motion_estimate_fails_closed_when_updates_become_stale():
    estimator = PoseMotionEstimator("camera_init", "body")
    update(estimator, 20.0, 20.0, 0.0, 0.0)
    estimate = update(estimator, 20.1, 20.1, 0.01, 0.0)

    assert motion_hold_reason(
        estimate, now_s=20.5, max_age_s=0.25) == "ODOM_STALE"


def test_stopping_distance_uses_measured_speed_and_sensor_latency():
    envelope = stopping_envelope(
        measured_speed_mps=0.8,
        requested_speed_mps=0.2,
        measured_yaw_rate_rps=0.1,
        requested_yaw_rate_rps=0.0,
        cloud_age_s=0.1,
        accumulation_s=1.0,
        pipeline_s=0.2,
        min_linear_decel_mps2=0.5,
        min_angular_decel_rps2=0.5,
        geometry_margin_m=0.9)

    assert envelope.speed_mps == pytest.approx(0.8)
    assert envelope.reaction_s == pytest.approx(1.3)
    assert envelope.distance_m == pytest.approx(2.58)
    assert envelope.horizon_s == pytest.approx(2.9)


def test_stopping_distance_never_shrinks_below_existing_geometry_margin():
    envelope = stopping_envelope(
        measured_speed_mps=0.0,
        requested_speed_mps=0.0,
        measured_yaw_rate_rps=0.0,
        requested_yaw_rate_rps=0.0,
        cloud_age_s=0.0,
        accumulation_s=1.0,
        pipeline_s=0.2,
        min_linear_decel_mps2=0.5,
        min_angular_decel_rps2=0.5,
        geometry_margin_m=0.9)

    assert envelope.distance_m == pytest.approx(0.9)


def test_rotation_sweep_detects_a_side_cluster_outside_forward_fov():
    side_cluster = np.array([
        [0.00, 0.50],
        [0.01, 0.50],
        [-0.01, 0.50],
        [0.00, 0.51],
        [0.00, 0.49],
    ])

    assert swept_footprint_collision(
        side_cluster,
        linear_speed_mps=0.0,
        angular_speed_rps=math.pi / 2.0,
        horizon_s=1.0,
        front_m=0.50,
        rear_m=0.50,
        half_width_m=0.30,
        margin_m=0.05,
        min_points=5)


def test_rotation_sweep_does_not_block_clear_space():
    far_cluster = np.array([
        [1.2, 1.2],
        [1.21, 1.2],
        [1.2, 1.21],
        [1.19, 1.2],
        [1.2, 1.19],
    ])

    assert not swept_footprint_collision(
        far_cluster,
        linear_speed_mps=0.0,
        angular_speed_rps=math.pi / 2.0,
        horizon_s=1.0,
        front_m=0.50,
        rear_m=0.50,
        half_width_m=0.30,
        margin_m=0.05,
        min_points=5)


def test_straight_sweep_does_not_mutate_side_distance_into_a_collision():
    clear_side_cluster = np.column_stack((
        np.linspace(0.70, 2.20, 50000),
        np.full(50000, 0.70)))

    assert not swept_footprint_collision(
        clear_side_cluster,
        linear_speed_mps=0.6,
        angular_speed_rps=0.0,
        horizon_s=2.5,
        front_m=0.50,
        rear_m=0.50,
        half_width_m=0.30,
        margin_m=0.15,
        min_points=5)


def test_rider_returns_are_removed_but_external_side_obstacles_remain():
    cloud = np.array([
        [-0.30, 0.00, 0.20],
        [-0.31, 0.01, 0.20],
        [-0.32, -0.01, 0.20],
        [-0.29, 0.02, 0.20],
        [-0.28, -0.02, 0.20],
        [0.00, 0.55, 0.20],
        [0.01, 0.55, 0.20],
        [-0.01, 0.55, 0.20],
        [0.00, 0.56, 0.20],
        [0.00, 0.54, 0.20],
    ])

    obstacles = filter_obstacle_points(
        cloud,
        sensor_height_m=0.30,
        min_height_m=0.15,
        max_height_m=1.9,
        self_x_min_m=-1.0,
        self_x_max_m=0.55,
        self_half_width_m=0.40)

    assert len(obstacles) == 5
    assert np.all(obstacles[:, 1] > 0.5)


def test_shared_collision_height_bounds_are_inclusive():
    sensor_height_m = 0.30
    cloud = np.array([
        [2.0, 0.0, COLLISION_MIN_HEIGHT_M - sensor_height_m],
        [3.0, 0.0, COLLISION_MAX_HEIGHT_M - sensor_height_m],
        [4.0, 0.0, COLLISION_MIN_HEIGHT_M - sensor_height_m - 0.01],
        [5.0, 0.0, COLLISION_MAX_HEIGHT_M - sensor_height_m + 0.01],
    ])

    obstacles = filter_obstacle_points(
        cloud,
        sensor_height_m=sensor_height_m,
        min_height_m=COLLISION_MIN_HEIGHT_M,
        max_height_m=COLLISION_MAX_HEIGHT_M,
        self_x_min_m=-1.0,
        self_x_max_m=0.55,
        self_half_width_m=0.40)

    assert obstacles[:, 0].tolist() == [2.0, 3.0]
