"""Thread-interleaving regressions for PRIEST command publication."""

from __future__ import annotations

import types

import numpy as np

from test_priest_follower_execution import (
    DriveCommand,
    Publisher,
    Stamp,
    follower_with,
    pf,
    plan_with,
)


def healthy_follower() -> pf.PriestFollower:
    follower = follower_with(plan_with())
    follower.pose_map = np.eye(4)
    follower.pose_stamp = Stamp(10.0)
    follower.prev_centre = follower.prev_stamp = None
    follower.motion = pf.MotionEstimate(True, 10.0, 10.0, 0.0, 0.0, "")
    follower.tracking_state = "TRACKING"
    follower.diag_stamp = Stamp(10.0)
    follower.degraded_since = None
    follower.wheel_status_stamp = Stamp(10.0)
    follower.drive_mode = 65
    follower.cluster_summary = types.SimpleNamespace(
        stamp_s=10.0, usable=True, objects=[])
    follower.band = types.SimpleNamespace(
        contains=lambda point, grace: True,
        contains_many=lambda points, grace: np.ones(len(points), dtype=bool))
    follower.lidar_in_body = np.zeros(3)
    follower.lidar_rotation = np.eye(3)
    follower.planner = types.SimpleNamespace(max_obstacles=24, a_max=0.18)
    follower.corridor = types.SimpleNamespace(
        centres=np.array([[0.0, 0.0], [5.0, 0.0]]),
        length_m=5.0, arc_of=lambda point: 0.0)
    follower.status = "PAUSED"
    follower.status_pub = Publisher()
    return follower


def test_disable_during_controller_work_cannot_publish_after_zero(
        monkeypatch) -> None:
    follower = follower_with(plan_with())

    def disable_during_command(*args, **kwargs) -> DriveCommand:
        follower.enabled = False
        follower.send_stop()
        return DriveCommand(0.4, 0.2)

    monkeypatch.setattr(pf, "command_for", disable_during_command)

    assert follower.track(Stamp(10.2)) == "PAUSED"
    command = follower.cmd_pub.messages[-1]
    assert command.linear.x == command.angular.z == 0.0


def test_malformed_usable_plan_times_fail_closed_before_progress_lookup() -> None:
    follower = follower_with(plan_with())
    follower.plan.times = None

    assert follower.unpredictable_reason(Stamp(10.2)) == "OBSTACLE_WAIT"


def test_malformed_usable_plan_points_fail_closed_before_drift() -> None:
    follower = healthy_follower()
    follower.plan.x = None

    assert follower.ensure_plan(Stamp(10.2)) == "INVALID_PLAN"
    assert follower.plan is None
    assert follower.cmd_pub.messages[-1].linear.x == 0.0


def test_non_numeric_usable_plan_arrays_fail_closed_everywhere() -> None:
    corrupt = plan_with()
    corrupt.x = np.array(["bad", "bad", "bad"])
    follower = healthy_follower()
    follower.plan = corrupt
    assert follower.ensure_plan(Stamp(10.2)) == "INVALID_PLAN"

    follower = healthy_follower()
    follower.plan = corrupt
    assert follower.unpredictable_reason(Stamp(10.2)) == "OBSTACLE_WAIT"
    assert follower.track(Stamp(10.2)) == "INVALID_PLAN"
    assert follower.cmd_pub.messages[-1].linear.x == 0.0


def test_non_numeric_certified_reference_fails_closed_in_controller() -> None:
    corrupt = plan_with()
    corrupt.velocity_xy_mps = np.full((3, 2), "bad")
    follower = follower_with(corrupt)

    assert follower.track(Stamp(10.2)) == "INVALID_PLAN"
    assert follower.cmd_pub.messages[-1].linear.x == 0.0


def test_overflowing_plan_and_reference_arrays_fail_closed() -> None:
    huge = np.array([10 ** 10000] * 3, dtype=object)
    corrupt = plan_with()
    corrupt.x = huge
    follower = healthy_follower()
    follower.plan = corrupt
    assert follower.ensure_plan(Stamp(10.2)) == "INVALID_PLAN"

    corrupt = plan_with()
    corrupt.yaw_rad = huge
    follower = follower_with(corrupt)
    assert follower.track(Stamp(10.2)) == "INVALID_PLAN"
    assert follower.cmd_pub.messages[-1].linear.x == 0.0


def test_local_plan_endpoint_does_not_latch_global_done() -> None:
    follower = follower_with(plan_with())
    follower.centre_xy = np.array([0.8, 0.0])
    follower.corridor = types.SimpleNamespace(
        centres=np.array([[0.0, 0.0], [5.0, 0.0]]))

    assert follower.track(Stamp(11.9)) == "PLAN_COMPLETE"
    assert not follower.done
    assert follower.cmd_pub.messages[-1].linear.x == 0.0


def test_empty_wheel_status_neither_refreshes_nor_clears_uncertainty() -> None:
    follower = healthy_follower()
    follower.wheel_status_stamp = Stamp(9.0)
    Stamp.current_s = 10.0

    follower.on_wheel_status(types.SimpleNamespace(data=[]))

    assert follower.wheel_status_stamp.to_sec() == 9.0
    assert follower.drive_mode is None
    assert follower.hold_reason(Stamp(10.0)) == "MANUAL_MODE"


def test_unknown_wheel_mode_cannot_authorize_motion() -> None:
    follower = healthy_follower()
    follower.drive_mode = None

    assert follower.hold_reason(Stamp(10.0)) == "MANUAL_MODE"


def test_future_pose_and_cluster_stamps_fail_closed() -> None:
    follower = healthy_follower()
    follower.pose_stamp = Stamp(999999.0)
    assert follower.hold_reason(Stamp(10.0)) == "NO_POSE"

    follower.pose_stamp = Stamp(10.0)
    follower.cluster_summary.stamp_s = 999999.0
    assert follower.hold_reason(Stamp(10.0)) == "CLUSTERS_STALE"


def test_missing_or_stale_localization_diagnostics_fail_closed() -> None:
    follower = healthy_follower()
    Stamp.current_s = 10.0

    follower.on_diag(types.SimpleNamespace(status=[]))

    assert follower.tracking_state == ""
    assert follower.hold_reason(Stamp(10.0)) \
        == "LOCALIZATION_NOT_TRACKING"
    follower.tracking_state = "TRACKING"
    follower.diag_stamp = Stamp(0.0)
    assert follower.hold_reason(Stamp(10.0)) \
        == "LOCALIZATION_NOT_TRACKING"
    tracking = types.SimpleNamespace(name="fast_lio_icp", message="TRACKING")
    follower.on_diag(types.SimpleNamespace(
        status=[tracking], header=types.SimpleNamespace(stamp=Stamp(0.0))))
    assert follower.hold_reason(Stamp(10.0)) \
        == "LOCALIZATION_NOT_TRACKING"


def test_disable_enable_during_planning_cannot_resurrect_candidate(
        monkeypatch) -> None:
    follower = healthy_follower()
    follower.plan = None
    responses = follower.on_start.__func__.__globals__
    monkeypatch.setitem(
        responses, "SetBoolResponse",
        lambda **values: types.SimpleNamespace(**values))

    def raced_plan(*args, **kwargs):
        follower.on_start(types.SimpleNamespace(data=False))
        follower.on_start(types.SimpleNamespace(data=True))
        return plan_with()

    follower.planner.plan = raced_plan

    assert follower.ensure_plan(Stamp(10.2)) == "PLAN_SUPERSEDED"
    assert follower.plan is None
    assert follower.cmd_pub.messages[-1].linear.x == 0.0


def test_static_obstacle_arriving_during_planning_rejects_candidate() -> None:
    follower = healthy_follower()
    follower.plan = None

    def raced_plan(*args, **kwargs):
        follower.cluster_summary = types.SimpleNamespace(
            stamp_s=10.1, usable=True, objects=[{
                "x": 0.4, "y": 0.0, "map_x": 0.4, "map_y": 0.0,
                "size": [0.2, 0.2, 1.0],
                "motion": "static",
            }])
        return plan_with()

    follower.planner.plan = raced_plan

    assert follower.ensure_plan(Stamp(10.2)) == "OBSTACLE_WAIT"
    assert follower.plan is None


def test_latest_pose_connector_is_recertified_against_static_obstacles() -> None:
    follower = healthy_follower()
    follower.centre_xy = np.array([0.0, 0.49])
    follower.cluster_summary = types.SimpleNamespace(
        stamp_s=10.1, usable=True, objects=[{
            "x": 0.0, "y": 1.10, "map_x": 0.0, "map_y": 1.10,
            "size": [0.2, 0.2, 1.0],
            "motion": "static",
        }])

    assert follower.static_plan_reason(follower.plan, 0.2) \
        == "OBSTACLE_WAIT"


def test_pose_jump_during_planning_rejects_stale_start() -> None:
    follower = healthy_follower()
    follower.plan = None

    def raced_plan(*args, **kwargs):
        follower.centre_xy = np.array([2.0, 0.0])
        return plan_with()

    follower.planner.plan = raced_plan

    assert follower.ensure_plan(Stamp(10.2)) == "PLAN_SUPERSEDED"
    assert follower.plan is None


def test_planner_numeric_error_becomes_zero_refusal() -> None:
    follower = healthy_follower()
    follower.plan = None

    def fail(*args, **kwargs):
        raise np.linalg.LinAlgError("singular")

    follower.planner.plan = fail

    assert follower.ensure_plan(Stamp(10.2)) == "PLAN_ERROR"
    assert follower.plan is None
    assert follower.cmd_pub.messages[-1].linear.x == 0.0


def test_invalid_pose_sample_atomically_revokes_pose_authority() -> None:
    follower = healthy_follower()
    follower.pose_correction = np.eye(4)
    follower.pose_frame = "map"
    vector = types.SimpleNamespace(x=1.0, y=2.0, z=0.0)
    zero_quaternion = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)
    message = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=Stamp(10.0), frame_id="map"),
        pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=vector, orientation=zero_quaternion)))

    follower.on_pose(message)

    assert follower.centre_xy is None and follower.pose_map is None
    assert follower.hold_reason(Stamp(10.0)) == "NO_POSE"


def test_existing_unpredictable_actor_blocks_before_replanning() -> None:
    follower = healthy_follower()
    follower.plan_stamp = Stamp(0.0)
    planned: list[bool] = []
    follower.planner.plan = lambda *args: planned.append(True) or plan_with()
    follower.unpredictable_reason = lambda now: "OBSTACLE_WAIT"
    Stamp.current_s = 10.2

    follower.step()

    assert planned == []
    assert follower.cmd_pub.messages[-1].linear.x == 0.0


def test_plan_age_does_not_cause_a_periodic_step_stop() -> None:
    follower = healthy_follower()
    follower.plan.x *= 10.0
    follower.plan.times *= 10.0
    follower.plan_stamp = Stamp(0.0)
    follower.previous_command = DriveCommand(0.30, 0.0)
    follower.motion = pf.MotionEstimate(
        True, 10.0, 10.0, 0.30, 0.0, "")
    planned: list[bool] = []

    follower.planner.plan = lambda *args, **kwargs: planned.append(True)
    Stamp.current_s = 10.2

    follower.step()

    command = follower.cmd_pub.messages[-1]
    assert not planned and command.linear.x > 0.0
    assert abs(command.linear.x - 0.30) / 0.2 <= 0.18 + 1e-9


def test_disable_between_command_and_status_cannot_report_driving() -> None:
    follower = healthy_follower()

    def disable_during_status(point):
        follower.enabled = False
        follower.send_stop()
        return 0.0

    follower.corridor.arc_of = disable_during_status
    Stamp.current_s = 10.2

    follower.step()

    assert follower.status_pub.messages[-1].data == "HOLD:PAUSED"
    assert follower.cmd_pub.messages[-1].linear.x == 0.0
