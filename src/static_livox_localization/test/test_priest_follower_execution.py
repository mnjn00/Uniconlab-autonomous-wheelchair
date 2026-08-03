"""Behavioral wiring checks for the certified PRIEST ROS adapter."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import numpy as np


ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT / "scripts"


class Stamp:
    current_s = 0.0

    def __init__(self, seconds: float = 0.0) -> None:
        self.seconds = float(seconds)

    @staticmethod
    def now() -> Stamp:
        return Stamp(Stamp.current_s)

    def __sub__(self, other: Stamp) -> Stamp:
        return Stamp(self.seconds - other.seconds)

    def to_sec(self) -> float:
        return self.seconds


class Vector:
    def __init__(self) -> None:
        self.x = self.y = self.z = 0.0


class Twist:
    def __init__(self) -> None:
        self.linear = Vector()
        self.angular = Vector()


class String:
    def __init__(self, data: str = "") -> None:
        self.data = data


class Publisher:
    def __init__(self) -> None:
        self.messages: list[Twist | String] = []

    def publish(self, message: Twist | String) -> None:
        self.messages.append(message)


def load_node():
    rospy = types.ModuleType("rospy")
    rospy.Time = Stamp
    for name in ("loginfo", "logwarn", "logwarn_throttle", "logerr"):
        setattr(rospy, name, lambda *args, **kwargs: None)
    modules = {"rospy": rospy}
    packages = {
        "diagnostic_msgs.msg": ["DiagnosticArray"],
        "geometry_msgs.msg": ["PoseWithCovarianceStamped", "Twist"],
        "nav_msgs.msg": ["Odometry"],
        "std_msgs.msg": ["Int16MultiArray", "String"],
        "std_srvs.srv": ["SetBool", "SetBoolRequest", "SetBoolResponse"],
    }
    for package, names in packages.items():
        module = types.ModuleType(package)
        for name in names:
            value = Twist if name == "Twist" else String if name == "String" \
                else type(name, (), {})
            setattr(module, name, value)
        modules[package] = module
        root, child = package.split(".")
        modules.setdefault(root, types.ModuleType(root))
        setattr(modules[root], child, module)
    transformations = types.ModuleType("tf.transformations")
    transformations.quaternion_matrix = lambda value: np.eye(4)
    modules["tf"] = types.ModuleType("tf")
    modules["tf"].transformations = transformations
    modules["tf.transformations"] = transformations
    saved = {name: sys.modules.get(name) for name in modules}
    for sibling in ("priest_follower_io", "priest_follower_planning"):
        sys.modules.pop(sibling, None)
    sys.modules.update(modules)
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            "priest_follower_execution_test", SCRIPTS / "priest_follower.py")
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))
        for name, prior in saved.items():
            if prior is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = prior
        for sibling in ("priest_follower_io", "priest_follower_planning"):
            sys.modules.pop(sibling, None)


pf = load_node()
sys.path.insert(0, str(SCRIPTS))
try:
    from priest_constraints import DEFAULT_CONSTRAINT_TOLERANCES
    from priest_controller import (
        DEFAULT_CONTROLLER_LIMITS, DriveCommand, command_for)
    from priest_feasibility import TrajectoryCertificate
    from priest_types import Plan
finally:
    sys.path.remove(str(SCRIPTS))


def plan_with(certified: bool = True) -> Plan:
    times = np.array([0.0, 1.0, 2.0])
    points = np.array([[0.0, 0.0], [0.4, 0.0], [0.8, 0.0]])
    certificate = TrajectoryCertificate.clear(
        DEFAULT_CONSTRAINT_TOLERANCES) if certified else None
    return Plan(
        np.zeros(6), points[:, 0], points[:, 1], times,
        0.0, 0.0, 1, 2.0, certificate=certificate,
        velocity_xy_mps=np.tile(np.array([0.4, 0.0]), (3, 1)),
        acceleration_xy_mps2=np.zeros((3, 2)),
        yaw_rad=np.zeros(3), yaw_rate_rps=np.zeros(3))


def follower_with(plan: Plan) -> pf.PriestFollower:
    follower = pf.PriestFollower.__new__(pf.PriestFollower)
    follower.plan = plan
    follower.plan_stamp = Stamp(10.0)
    follower.command_lock = pf.threading.RLock()
    follower.control_epoch = 0
    follower.centre_xy = np.zeros(2)
    follower.pose_yaw = 0.0
    follower.velocity = np.zeros(2)
    follower.motion = types.SimpleNamespace(linear_speed_mps=0.0)
    follower.previous_command = DriveCommand(0.0, 0.0)
    follower.enabled = True
    follower.done = False
    follower.controller_limits = DEFAULT_CONTROLLER_LIMITS
    follower.current_speed = follower.last_yaw_rate = 0.0
    follower.cmd_pub = Publisher()
    follower.band = types.SimpleNamespace(
        contains_many=lambda points, grace: np.ones(len(points), dtype=bool))
    follower.cluster_summary = types.SimpleNamespace(usable=True, objects=[])
    follower.pose_map, follower.lidar_rotation = np.eye(4), np.eye(3)
    follower.lidar_in_body = np.zeros(3)
    follower.planner = types.SimpleNamespace(max_obstacles=24, a_max=0.18)
    return follower


def ready_step(follower: pf.PriestFollower) -> None:
    follower.tracking_state, follower.status = "", "PAUSED"
    follower.degraded_since = None
    follower.status_pub = Publisher()
    follower.hold_reason = lambda now: None
    follower.unpredictable_reason = lambda now: None
    follower.ensure_plan = lambda now: None
    follower.static_plan_reason = lambda plan, elapsed=0.0: None
    follower.corridor = types.SimpleNamespace(
        centres=np.array([[0.0, 0.0], [5.0, 0.0]]), length_m=5.0,
        arc_of=lambda point: 0.0)


def test_track_passes_plan_elapsed_time_to_actual_controller(monkeypatch) -> None:
    follower = follower_with(plan_with())
    observed: list[float] = []

    def recording(plan, elapsed_s, *args, **kwargs):
        observed.append(elapsed_s)
        return command_for(plan, elapsed_s, *args, **kwargs)

    monkeypatch.setattr(pf, "command_for", recording, raising=False)
    reason = follower.track(Stamp(11.25))

    assert reason is None
    assert observed == [1.25]
    assert follower.cmd_pub.messages[-1].linear.x > 0.0


def test_track_refuses_an_uncertified_plan_with_exact_zero() -> None:
    follower = follower_with(plan_with(certified=False))

    reason = follower.track(Stamp(10.2))

    assert reason == "UNCERTIFIED_PLAN"
    command = follower.cmd_pub.messages[-1]
    assert command.linear.x == command.angular.z == 0.0


def test_send_stop_resets_plan_and_controller_state() -> None:
    follower = follower_with(plan_with())
    follower.previous_command = DriveCommand(0.3, 0.2)
    follower.current_speed = 0.3
    follower.last_yaw_rate = 0.2

    follower.send_stop()

    assert follower.plan is None
    assert follower.previous_command == DriveCommand(0.0, 0.0)
    assert follower.current_speed == follower.last_yaw_rate == 0.0
    assert follower.cmd_pub.messages[-1].linear.x == 0.0


def test_step_hold_publishes_zero_and_resets_state() -> None:
    follower = follower_with(plan_with())
    ready_step(follower)
    follower.status = "DRIVING"
    follower.hold_reason = lambda now: "OFF_BAND"
    Stamp.current_s = 12.0

    follower.step()

    assert follower.status_pub.messages[-1].data == "HOLD:OFF_BAND"
    assert follower.previous_command == DriveCommand(0.0, 0.0)
    assert follower.plan is None


def test_step_executes_actual_timed_controller_and_reports_driving() -> None:
    follower = follower_with(plan_with())
    ready_step(follower)
    Stamp.current_s = 10.2

    follower.step()

    assert follower.cmd_pub.messages[-1].linear.x > 0.0
    assert follower.status_pub.messages[-1].data.startswith("DRIVING ")


def test_step_goal_is_done_with_exact_zero_and_reset() -> None:
    follower = follower_with(plan_with())
    ready_step(follower)
    follower.corridor.centres[-1] = np.array([0.01, 0.0])
    Stamp.current_s = 10.2

    follower.step()

    assert follower.done
    assert follower.status_pub.messages[-1].data == "HOLD:DONE"
    assert follower.plan is None
    assert follower.cmd_pub.messages[-1].linear.x == 0.0


def test_step_exact_goal_tolerance_boundary_latches_done() -> None:
    follower = follower_with(plan_with())
    ready_step(follower)
    follower.centre_xy = np.array([1.05, 0.0])
    follower.corridor.centres[-1] = np.array([1.0, 0.0])
    Stamp.current_s = 10.2

    follower.step()

    assert follower.done
    assert follower.status_pub.messages[-1].data == "HOLD:DONE"


def test_step_checks_unpredictable_against_newly_accepted_plan() -> None:
    follower = follower_with(plan_with())
    ready_step(follower)
    accepted = plan_with()
    checked: list[bool] = []
    follower.ensure_plan = lambda now: setattr(follower, "plan", accepted)
    follower.unpredictable_reason = lambda now: (
        checked.append(follower.plan is accepted)
        or (None if len(checked) == 1 else "OBSTACLE_WAIT"))
    Stamp.current_s = 10.2

    follower.step()

    assert checked == [False, True]
    assert follower.status_pub.messages[-1].data == "HOLD:OBSTACLE_WAIT"


def test_step_rechecks_guards_after_planning_before_command() -> None:
    follower = follower_with(plan_with())
    ready_step(follower)
    checks: list[float] = []

    def guard(now: Stamp) -> str | None:
        checks.append(now.to_sec())
        return None if len(checks) == 1 else "CLUSTERS_STALE"

    follower.hold_reason = guard
    Stamp.current_s = 10.2

    follower.step()

    assert len(checks) == 2
    assert follower.status_pub.messages[-1].data == "HOLD:CLUSTERS_STALE"
    assert follower.cmd_pub.messages[-1].linear.x == 0.0


def test_unpredictable_policy_gets_time_derived_plan_progress(monkeypatch) -> None:
    follower = follower_with(plan_with())
    follower.cluster_summary = types.SimpleNamespace()
    follower.pose_map = np.eye(4)
    follower.lidar_in_body = np.zeros(3)
    follower.lidar_rotation = np.eye(3)
    captured: list[int] = []

    def wait_reason(*args, **kwargs):
        captured.append(kwargs["trajectory_start_index"])
        return None

    monkeypatch.setattr(pf, "runtime_wait_reason", wait_reason, raising=False)

    assert follower.unpredictable_reason(Stamp(11.1)) is None
    assert captured == [1]
