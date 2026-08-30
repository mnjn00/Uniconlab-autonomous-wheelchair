import types

import numpy as np
import pytest

from test_dwa_policy import load_follower


def object_follower():
    module, Stamp = load_follower("person_bypass_dwa_follower")
    follower = module.PersonBypassDwaFollower.__new__(
        module.PersonBypassDwaFollower)
    follower._committed_bypass_track_id = 3163
    follower._committed_bypass_side = 1
    follower._committed_bypass_kind = "object"
    follower._static_object_track_id = 3163
    follower._static_object_world_xy = (5.0, 0.0)
    follower._static_object_half_forward_m = 0.4
    follower._static_object_half_lateral_m = 0.4
    follower._static_object_passed_side = False
    follower._static_object_commit_until_s = 101.5
    follower._post_pass_track_id = None
    follower._post_pass_origin_xy = None
    follower.active_trajectory_permit = None
    follower.static_object_reacquire_radius_m = 1.5
    follower.static_object_slowdown_distance_m = 3.0
    follower.person_bypass_minimum_near_m = 0.6
    follower.person_bypass_clearance_m = 0.6
    follower.person_bypass_speed_mps = 0.35
    follower.planner = types.SimpleNamespace(max_speed=0.8)
    return module, Stamp, follower


def test_dropout_grace_keeps_side_but_does_not_create_raw_authorization():
    _module, Stamp, follower = object_follower()
    veto = follower.planner_candidate_veto(
        Stamp(100.0), None, lambda v, w: (v, w))
    assert veto is not None
    assert not veto(0.35, 0.2)
    assert not veto(0.35, 0.0)
    assert veto(0.35, -0.2)


def test_commitment_survives_only_the_bounded_grace():
    _module, _Stamp, follower = object_follower()
    follower.reset_bypass_commitment(now_s=100.0)
    assert follower._committed_bypass_side == 1
    follower.reset_bypass_commitment(now_s=102.0)
    assert follower._committed_bypass_side == 0
    assert follower._static_object_world_xy is None


def test_new_track_near_the_map_anchor_reacquires_same_bicycle():
    _module, _Stamp, follower = object_follower()
    follower.pose_xy = np.array([0.0, 0.0])
    follower.pose_yaw = 0.0
    threat = types.SimpleNamespace(
        track_id=3194, distance_m=4.8, lateral_m=0.2,
        centre_forward_m=5.1, half_forward_m=0.3,
        half_lateral_m=0.35)
    assert not follower.update_static_object_pass(threat)
    assert follower._static_object_track_id == 3194
    assert follower._committed_bypass_track_id == 3194
    assert follower._committed_bypass_side == 1
    assert follower._static_object_world_xy == pytest.approx((5.0, 0.0))


def test_static_object_speed_is_cruise_far_and_slow_only_inside_three_metres():
    _module, _Stamp, follower = object_follower()
    assert follower.static_object_permit_speed_mps(
        types.SimpleNamespace(distance_m=6.0)) == pytest.approx(0.8)
    assert follower.static_object_permit_speed_mps(
        types.SimpleNamespace(distance_m=2.9)) == pytest.approx(0.35)
