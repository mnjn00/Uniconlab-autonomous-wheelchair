import importlib.util
import json
import sys
import types
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load_follower(name):
    dummy = type("Dummy", (), {})

    class Stamp(object):
        def __init__(self, seconds=0.0):
            self.seconds = float(seconds)

        def to_sec(self):
            return self.seconds

        def __sub__(self, other):
            return Stamp(self.seconds - other.seconds)

    rospy = types.ModuleType("rospy")
    rospy.loginfo = rospy.logwarn = lambda *args, **kwargs: None
    rospy.logwarn_throttle = rospy.logerr_throttle = lambda *args, **kwargs: None
    rospy.Time = type("Time", (), {"now": staticmethod(lambda: Stamp(100.0))})
    rospy.get_param = lambda *args, **kwargs: args[1] if len(args) > 1 else None

    class Twist(object):
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class String(object):
        def __init__(self, data=""):
            self.data = data

    modules = {"rospy": rospy}
    for package, names in {
        "diagnostic_msgs.msg": ["DiagnosticArray"],
        "geometry_msgs.msg": ["PoseWithCovarianceStamped"],
        "nav_msgs.msg": ["Odometry"],
        "sensor_msgs.msg": ["PointCloud2"],
        "std_msgs.msg": ["Int16MultiArray", "String"],
        "std_srvs.srv": ["SetBool", "SetBoolResponse"],
    }.items():
        module = types.ModuleType(package)
        for entry in names:
            setattr(module, entry, dummy)
        modules[package] = module
        root = package.split(".")[0]
        modules.setdefault(root, types.ModuleType(root))
        setattr(modules[root], package.split(".")[1], module)
    modules["geometry_msgs.msg"].Twist = Twist
    modules["std_msgs.msg"].String = String
    point_cloud = types.ModuleType("sensor_msgs.point_cloud2")
    point_cloud.read_points = lambda *args, **kwargs: []
    modules["sensor_msgs.point_cloud2"] = point_cloud
    modules["sensor_msgs"].point_cloud2 = point_cloud
    transformations = types.ModuleType("tf.transformations")
    transformations.quaternion_matrix = lambda value: np.eye(4)
    transformations.euler_from_quaternion = lambda value: (0.0, 0.0, 0.0)
    modules["tf"] = types.ModuleType("tf")
    modules["tf"].transformations = transformations
    modules["tf.transformations"] = transformations

    saved = {key: sys.modules.get(key) for key in modules}
    sys.modules.update(modules)
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            "static_threat_follower_test_" + name, SCRIPTS / (name + ".py"))
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module, Stamp, Twist
    finally:
        sys.path.remove(str(SCRIPTS))
        for key, cached in list(sys.modules.items()):
            origin = getattr(cached, "__file__", None)
            if origin and Path(origin).parent == SCRIPTS:
                del sys.modules[key]
        for key, value in saved.items():
            if value is None:
                sys.modules.pop(key, None)
            else:
                sys.modules[key] = value


def test_qualification_updates_once_for_each_summary_stamp(monkeypatch):
    module, Stamp, _Twist = load_follower("person_bypass_dwa_follower")
    follower = module.PersonBypassDwaFollower.__new__(
        module.PersonBypassDwaFollower)
    calls = []
    permit = types.SimpleNamespace(active=False)
    follower.qualifier = types.SimpleNamespace(
        update=lambda *args, **kwargs: calls.append((args, kwargs)) or permit)
    follower.cluster_summary = types.SimpleNamespace(usable=True, stamp_s=10.0)
    follower.tracking_state = "TRACKING"
    follower.person_bypass_maximum_forward_m = 8.0
    follower.person_bypass_maximum_lateral_m = 1.0
    follower.person_bypass_lateral_hysteresis_m = 0.25
    monkeypatch.setattr(module, "threat_observations", lambda *args, **kwargs: ())

    first = follower.observed_threat_permit(Stamp(10.1))
    second = follower.observed_threat_permit(Stamp(10.2))

    assert first is second
    assert len(calls) == 1


def test_gate_clear_release_requires_matching_proposal_and_three_cycles():
    module, _Stamp, _Twist = load_follower("person_bypass_dwa_follower")
    follower = module.PersonBypassDwaFollower.__new__(
        module.PersonBypassDwaFollower)
    observed = []
    clear_count = [0]

    def observe_tail_clear(clear):
        observed.append(clear)
        clear_count[0] = clear_count[0] + 1 if clear else 0
        return clear_count[0] >= 3

    follower.qualifier = types.SimpleNamespace(
        committed=True, pass_side="left", track_id=7,
        observe_tail_clear=observe_tail_clear)
    follower.active_proposal_seq = 11
    follower._latest_permit = types.SimpleNamespace(track_id=7)
    follower._clear_released = False

    mismatched = {
        "static_threat_target_behind": True,
        "static_threat_tail_clear": True,
        "trajectory_proposal_seq": 10,
        "static_threat_bypass_track_id": 7,
    }
    follower.consume_bypass_gate_report(mismatched)
    for _ in range(3):
        follower.consume_bypass_gate_report(dict(
            mismatched, trajectory_proposal_seq=11))

    assert observed == [False, True, True, True]
    assert follower._clear_released is True


def test_accepted_zero_keeps_committed_side():
    module, _Stamp, Twist = load_follower("person_bypass_dwa_follower")
    follower = module.PersonBypassDwaFollower.__new__(
        module.PersonBypassDwaFollower)
    follower.current_speed = 0.35
    follower.last_yaw_rate = 0.2
    follower.command_accel = 0.18
    follower.qualifier = types.SimpleNamespace(committed=True, pass_side="right")
    command = Twist()

    follower.on_accepted_command(command)

    assert follower.current_speed == 0.0
    assert follower.command_accel == 0.0
    assert follower.qualifier.committed is True
    assert follower.qualifier.pass_side == "right"


def test_publish_proposal_immediately_before_matching_command():
    module, _Stamp, Twist = load_follower("dwa_follower")
    events = []
    proposal = types.SimpleNamespace(
        first_applied_speed_mps=0.35,
        first_applied_yaw_rate_rps=0.15,
        to_json=lambda: json.dumps({"proposal_seq": 4}))
    follower = module.DwaFollower.__new__(module.DwaFollower)
    follower.proposal_pub = types.SimpleNamespace(
        publish=lambda message: events.append(("proposal", json.loads(message.data))))
    follower.cmd_pub = types.SimpleNamespace(
        publish=lambda message: events.append(
            ("command", message.linear.x, message.angular.z)))

    follower.publish_proposal_command(proposal, Twist)

    assert events == [
        ("proposal", {"proposal_seq": 4}),
        ("command", 0.35, 0.15),
    ]


def test_raw_only_gate_stall_remains_stop_only():
    module, Stamp, _Twist = load_follower("person_bypass_dwa_follower")
    follower = module.PersonBypassDwaFollower.__new__(
        module.PersonBypassDwaFollower)
    follower._latest_permit = types.SimpleNamespace(active=True, track_id=7)

    assert follower.may_bypass_gate_stall(Stamp(4.0), None) is False


@pytest.mark.parametrize("label", ["person", "obstacle"])
def test_person_and_object_wait_at_1_8_and_bypass_at_2_0(label):
    module, _Stamp, _Twist = load_follower("person_bypass_policy")
    manager = module.StaticThreatBypassManager()

    def observation(stamp):
        return module.StaticThreatObservation(
            track_id=7, stamp_s=stamp, x_m=1.5, y_m=0.0,
            size_x_m=0.5, size_y_m=0.5, label=label,
            motion="static", source="geometric")

    waiting = None
    for stamp in np.arange(0.0, 2.0, 0.2):
        waiting = manager.update(
            (observation(float(stamp)),), float(stamp), True,
            summary_stamp_s=float(stamp))
    active = manager.update(
        (observation(2.0),), 2.0, True, summary_stamp_s=2.0)

    assert waiting.active is False
    assert active.active is True
    assert active.track_id == 7
    assert active.threat_label == label


def test_one_dropout_keeps_committed_permit_but_second_does_not():
    module, _Stamp, _Twist = load_follower("person_bypass_policy")
    manager = module.StaticThreatBypassManager()
    first = module.StaticThreatObservation(
        7, 0.0, 1.5, 0.0, 0.5, 0.5,
        "person", "static", "geometric")
    second = module.StaticThreatObservation(
        7, 2.0, 1.5, 0.0, 0.5, 0.5,
        "person", "static", "geometric")
    manager.update((first,), 0.0, True, summary_stamp_s=0.0)
    for stamp in np.arange(0.2, 2.0, 0.2):
        tracked = module.StaticThreatObservation(
            7, float(stamp), 1.5, 0.0, 0.5, 0.5,
            "person", "static", "geometric")
        manager.update(
            (tracked,), float(stamp), True,
            summary_stamp_s=float(stamp))
    manager.update((second,), 2.0, True, summary_stamp_s=2.0)

    grace = manager.update((), 2.2, True, summary_stamp_s=2.2)
    stopped = manager.update((), 2.4, True, summary_stamp_s=2.4)

    assert grace.active is True
    assert stopped.active is False


def test_moving_observation_stops_committed_bypass_immediately():
    module, _Stamp, _Twist = load_follower("person_bypass_policy")
    manager = module.StaticThreatBypassManager()
    initial = module.StaticThreatObservation(
        7, 0.0, 1.5, 0.0, 0.5, 0.5,
        "person", "static", "geometric")
    qualified = module.StaticThreatObservation(
        7, 2.0, 1.5, 0.0, 0.5, 0.5,
        "person", "static", "geometric")
    moving = module.StaticThreatObservation(
        7, 2.2, 1.5, 0.0, 0.5, 0.5,
        "person", "moving", "geometric")
    manager.update((initial,), 0.0, True, summary_stamp_s=0.0)
    for stamp in np.arange(0.2, 2.0, 0.2):
        tracked = module.StaticThreatObservation(
            7, float(stamp), 1.5, 0.0, 0.5, 0.5,
            "person", "static", "geometric")
        manager.update(
            (tracked,), float(stamp), True,
            summary_stamp_s=float(stamp))
    manager.update((qualified,), 2.0, True, summary_stamp_s=2.0)

    permit = manager.update(
        (moving,), 2.2, True, dynamic_conflict=True,
        summary_stamp_s=2.2)

    assert permit.active is False
    assert permit.reason == "DYNAMIC_CONFLICT"


def test_matching_tracked_threat_may_bypass_gate_stall(monkeypatch):
    module, Stamp, _Twist = load_follower("person_bypass_dwa_follower")
    follower = module.PersonBypassDwaFollower.__new__(
        module.PersonBypassDwaFollower)
    permit = types.SimpleNamespace(active=True, track_id=7)
    threat = types.SimpleNamespace(track_id=7)
    follower._latest_permit = permit
    monkeypatch.setattr(
        module, "permit_matches_threat",
        lambda candidate, current, now_s: (
            candidate is permit and current is threat and now_s == 4.0))

    assert follower.may_bypass_gate_stall(Stamp(4.0), threat) is True


def test_bypass_planner_requires_curved_candidate_even_when_straight_is_cheapest():
    module, _Stamp, _Twist = load_follower("dwa_follower")
    calls = []
    curved = types.SimpleNamespace(target_yaw_rate_rps=0.1)

    def plan(*args, **kwargs):
        calls.append(dict(kwargs, state=args[0]))
        if kwargs.get("minimum_turn_rps", 0.0) >= 0.08:
            return 0.35, 0.1, "OK", curved
        return 0.35, 0.0, "OK", types.SimpleNamespace(
            target_yaw_rate_rps=0.0)

    follower = module.DwaFollower.__new__(module.DwaFollower)
    follower.planner = types.SimpleNamespace(plan=plan)
    follower.current_speed = 0.2
    follower.last_yaw_rate = -0.1
    follower.latency_s = 0.55
    actuator = module.ActuatorState(0.2, -0.1, 0.05, 0.1)
    anchored = np.array([4.0, -2.0, 0.4, 0.2, -0.1])
    follower.led_state = lambda state: (_ for _ in ()).throw(
        AssertionError("bypass must not pre-lead anchored state"))

    _v, yaw, status, proposal = follower.request_bypass_proposal(
        follower.planner_state(anchored, (7, None)),
        (), 0.35, actuator, 7, None, 1, 2.0)

    assert calls[0]["minimum_turn_rps"] == 0.08
    assert calls[0]["latency_s"] == 0.55
    assert calls[0]["actuator_state"] == actuator
    assert np.array_equal(calls[0]["state"], anchored)
    assert status == "OK"
    assert yaw == 0.1
    assert proposal is curved
