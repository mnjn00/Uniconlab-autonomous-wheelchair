"""Parked or moving, asked by the two profiles that replace step().

test_follower_avoidance runs this policy through the pursuit node. This runs
it through the OTHER two, because until 2026-08-11 neither of them asked the
question at all - it lived in the body of pursuit's step(), and dwa_follower
and mpc_follower replace step() entirely. Nothing drifted; the guard was
simply not carried across, and the profile that would stand and wait for
someone walking picked an arc round them instead.

The second half is what it goes round WITH: every lateral slice the object
occupies rather than its nearest return alone. test_object_profile pins that
against the 2026-07-31 wall; this pins that the follower actually passes it.
"""

import importlib.util
import json
import sys
import types
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    """Load a script module by path, with its siblings importable."""
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            name, SCRIPTS / ("%s.py" % name))
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


cg = load("cluster_guard")
ct = load("cluster_tracking")


# ------------------------------------------------ the profile's own policy

def load_follower(name):
    """The follower modules, with the ROS surface they import stubbed out."""
    dummy = type("Dummy", (), {})

    class Stamp(object):
        def __init__(self, seconds=0.0):
            self.seconds = float(seconds)

        def to_sec(self):
            return self.seconds

        def __sub__(self, other):
            return Stamp(self.seconds - other.seconds)

    rospy = types.ModuleType("rospy")
    rospy.loginfo = rospy.logwarn = lambda *a, **k: None
    rospy.logwarn_throttle = rospy.logerr_throttle = lambda *a, **k: None
    rospy.Time = type("Time", (), {"now": staticmethod(lambda: Stamp(100.0))})
    rospy.get_param = lambda *a, **k: (a[1] if len(a) > 1 else None)

    class Twist(object):
        """Real enough to be filled in - the command is what is asserted."""

        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

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
    point_cloud = types.ModuleType("sensor_msgs.point_cloud2")
    point_cloud.read_points = lambda *a, **k: []
    modules["sensor_msgs.point_cloud2"] = point_cloud
    modules["sensor_msgs"].point_cloud2 = point_cloud
    transformations = types.ModuleType("tf.transformations")
    transformations.quaternion_matrix = lambda value: np.eye(4)
    transformations.euler_from_quaternion = lambda value: (0.0, 0.0, 0.0)
    tf = types.ModuleType("tf")
    tf.transformations = transformations
    modules["tf"] = tf
    modules["tf.transformations"] = transformations

    saved = {key: sys.modules.get(key) for key in modules}
    sys.modules.update(modules)
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            "dwa_policy_test_" + name, SCRIPTS / ("%s.py" % name))
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        module.rospy.Time = rospy.Time
        return module, Stamp
    finally:
        sys.path.remove(str(SCRIPTS))
        # The follower modules import each other by plain name, so loading
        # one leaves waypoint_follower and its siblings CACHED - and cached
        # having been built against the stubs above. tests/
        # test_mpc_vehicle_layer.py then gets what this file left behind
        # instead of loading its own, and fails somewhere with no clue as to
        # why. Everything that came out of the scripts directory goes back
        # out; the directory itself stays on sys.path, because the scripts
        # put it there themselves and mpc_speed imports mpc_core lazily,
        # inside a call, long after any loader has finished.
        for key, cached in list(sys.modules.items()):
            origin = getattr(cached, "__file__", None)
            if origin and Path(origin).parent == SCRIPTS:
                del sys.modules[key]
        for key, value in saved.items():
            if value is None:
                sys.modules.pop(key, None)
            else:
                sys.modules[key] = value


class RecordingPlanner(object):
    """Stands in for DwaPlanner so the test can see what it was told."""

    def __init__(self):
        self.calls = []

    def plan(self, state, obstacles=(), speed_cap=None,
             last_yaw_rate=0.0, last_speed=None):
        self.calls.append({"obstacles": list(obstacles),
                           "speed_cap": speed_cap})
        return 0.3, 0.0, "OK"


def dwa_with(objects, monkeypatch, threat_distance_stop_radius=1.5):
    """A DwaFollower whose collaborators are visible to the test.

    monkeypatch rather than plain assignment, because mpc_speed is a shared
    module: replacing shaped_reference on it outright leaves every later
    test in the session calling this file's stub, and the one that notices
    is tests/test_mpc_vehicle_layer.py failing on a number with no
    connection to anything here.
    """
    module, Stamp = load_follower("dwa_follower")
    follower = module.DwaFollower.__new__(module.DwaFollower)
    published, commanded = [], []

    follower.planner = RecordingPlanner()
    follower.clusters_enabled = True
    follower.policies = True
    follower.cluster_summary = cg.parse_summary(json.dumps(
        {"stamp": 100.0, "status": "OK", "objects": objects}))
    follower.blocked_since = None
    follower.lateral_offset = 0.0
    follower.pose_xy = np.array([10.0, 0.0])
    follower.pose_yaw = 0.0
    follower.pose_pitch = 0.0
    follower.tracking_state = "TRACKING"
    follower.nearest_index = 5
    follower.waypoints = np.column_stack(
        [np.arange(0.0, 40.0, 0.2), np.zeros(200)])
    follower.current_speed = 0.0
    follower.latency_s = 0.30
    follower.odom_v = 0.0
    follower.odom_w = 0.0
    follower.last_yaw_rate = 0.0
    follower.last_command_stamp = None
    follower.dwa_status = ""
    follower.command_accel = 0.0
    # What safety_gate last said. Nothing by default: a fixture that
    # silently claimed the gate was blocking would put every test in this
    # file down the stall branch.
    follower.gate_reason = ""
    follower.gate_blocked_since = None
    follower.gate_detail = ""
    follower.gate_blocked_for = lambda now: None
    # The base's own report is the only velocity on the bus; the double
    # stands in for it because /Odometry carries no twist.
    follower.measured_speed = 0.0
    follower.measured_yaw_rate = 0.0
    follower.status = ""
    follower.band = None
    follower.anchor = types.SimpleNamespace(
        update=lambda *a, **k: np.array([10.0, 0.0, 0.0, 0.0, 0.0]),
        reset=lambda *a, **k: None)
    follower.cmd_pub = types.SimpleNamespace(
        publish=lambda message: commanded.append(message))
    follower.status_pub = types.SimpleNamespace(publish=lambda message: None)

    # Not what is under test, and each one reaches outside this process.
    follower.advance_progress = lambda: None
    follower.handled_before_driving = lambda now: False
    follower.send_stop = lambda: commanded.append("STOP")
    follower.publish_state = lambda text, state=None: published.append(text)
    # The braking envelope has its own tests; this file is about which
    # branch the decision takes, so the radius is a fixture and not a
    # measurement.
    follower.stop_radius = lambda: threat_distance_stop_radius
    monkeypatch.setattr(module.mpc_speed, "shaped_reference",
                        lambda *a, **k: (np.array([0.6, 0.6]), None))
    return module, follower, published, commanded


def parked(x, y=0.0, size=(0.6, 0.6, 1.2)):
    return {"class": "obstacle", "x": x, "y": y, "size": list(size),
            "points": 40, "motion": ct.STATIC}


def walking(x, y=0.0, size=(0.6, 0.6, 1.7)):
    return {"class": "person", "x": x, "y": y, "size": list(size),
            "points": 40, "motion": ct.MOVING}


def test_the_dwa_profile_waits_for_someone_walking(monkeypatch):
    """The defect, as the behaviour it produced: a rollout scorer handed a
    moving person picks the arc that clears them by OBSTACLE_FLOOR_M and
    drives past. Stepping around someone is a manoeuvre into where they are
    about to be, and this profile now refuses it like the other two."""
    _module, follower, published, commanded = dwa_with([walking(1.0)], monkeypatch)

    follower.step()

    assert published == ["HOLD:DWA_WAIT"]
    assert commanded == ["STOP"]
    assert follower.planner.calls == [], \
        "the planner was asked to find a way round a person"


def test_a_person_gets_the_wider_055m_stop_corridor(monkeypatch):
    """The 08-24 drive saw a walking person at y=0.8 m with a 0.6 m-wide
    box.  Its near flank was 0.50 m from the centreline: outside the ordinary
    0.45 m obstacle corridor, but inside the operator-selected 0.55 m person
    stop corridor.  It must be waited out rather than handed to the planner.
    """
    _module, follower, published, commanded = dwa_with(
        [walking(1.0, y=0.8)], monkeypatch)

    follower.step()

    assert published == ["HOLD:DWA_WAIT"]
    assert commanded == ["STOP"]
    assert follower.planner.calls == []


def test_a_person_stops_at_120_percent_of_the_dynamic_radius(monkeypatch):
    """At x=2.0 m the 0.6 m-long box starts 1.7 m ahead.  That is beyond
    the 1.5 m base radius but inside its 1.8 m person-only extension.
    """
    _module, follower, published, commanded = dwa_with(
        [walking(2.0)], monkeypatch, threat_distance_stop_radius=1.5)

    follower.step()

    assert published == ["HOLD:DWA_WAIT"]
    assert commanded == ["STOP"]
    assert follower.planner.calls == []


def test_the_person_extensions_do_not_widen_an_ordinary_moving_object(
        monkeypatch):
    moving_object = parked(2.0, y=0.8)
    moving_object["motion"] = ct.MOVING
    _module, follower, published, _commanded = dwa_with(
        [moving_object], monkeypatch, threat_distance_stop_radius=1.5)

    follower.step()

    assert not any(text.startswith("HOLD") for text in published)
    assert len(follower.planner.calls) == 1


def test_it_does_not_sidestep_someone_it_has_not_yet_had_to_stop_for(monkeypatch):
    """Further away than the stop radius the answer is CLEAR, not GO_ROUND.
    The chair keeps driving - but the planner is given no object to bend
    around, because bending around this one is never authorised."""
    _module, follower, published, _commanded = dwa_with([walking(3.0)], monkeypatch)

    follower.step()

    assert not any(text.startswith("HOLD") for text in published)
    assert follower.planner.calls[0]["obstacles"] == []


def test_it_goes_round_what_the_tracker_has_watched_stand_still(monkeypatch):
    _module, follower, _published, _commanded = dwa_with([parked(3.0)], monkeypatch)

    follower.step()

    assert follower.planner.calls[0]["obstacles"], \
        "a parked object still has to be planned around"


def test_what_it_goes_round_arrives_as_a_shape(monkeypatch):
    """One point per lateral slice the object occupies. A wall handed over
    as its nearest return alone admits an arc through the rest of it."""
    wide = parked(3.0, size=(0.6, 1.6, 1.2))
    _module, follower, _published, _commanded = dwa_with([wide], monkeypatch)

    follower.step()

    points = follower.planner.calls[0]["obstacles"]
    assert len(points) >= 7
    lateral = [float(p[1]) for p in points]
    assert max(lateral) - min(lateral) > 1.0


def test_the_approach_slows_the_way_the_pursuit_profile_slows(monkeypatch):
    """A planner that only knows stop-or-cruise arrives at what it is about
    to wait for at full speed."""
    # Both outside the person stop radius, which is not the plain radius:
    # stop_radius_for scales it by PERSON_STOP_DISTANCE_SCALE for a
    # person-labelled threat, so at the fixture's 1.5 m the decision point
    # is 1.8 m and a person at 2.0 m - near face 1.7 m - is waited for
    # rather than approached. This test is about the approach ramp, so it
    # asks about two distances that are both still an approach.
    _module, near, _p, _c = dwa_with([walking(2.6)], monkeypatch)
    _module, far, _p2, _c2 = dwa_with([walking(6.0)], monkeypatch)

    near.step()
    far.step()

    assert near.planner.calls[0]["speed_cap"] < \
        far.planner.calls[0]["speed_cap"]


def test_a_silent_producer_holds_this_profile_too(monkeypatch):
    """cluster_threat reports a missing summary as blocking at zero, which
    is WAIT and not clear road."""
    _module, follower, published, _commanded = dwa_with([], monkeypatch)
    follower.cluster_summary = None

    follower.step()

    assert published == ["HOLD:DWA_WAIT"]


def test_both_replacement_profiles_ask_the_shared_policy(monkeypatch):
    """The property this file exists for. A profile that replaces step()
    has to reach the same decision through the same function - a second
    copy drifts, and an omitted one already did."""
    for name in ("dwa_follower", "mpc_follower"):
        text = (SCRIPTS / ("%s.py" % name)).read_text(encoding="utf-8")
        assert "self.avoidance_for(" in text, name
        assert "== WAIT" in text, name
        assert "avoidance_decision(" not in text, \
            "%s must not re-implement the decision" % name


# ------------------------------------------- the gate refusing what we cannot see

def blocking_gate(follower, reason="OBSTACLE", held_s=2.0,
                  detail="OBSTACLE at 1.6 m, envelope 2.6 m"):
    follower.gate_reason = reason
    follower.gate_detail = detail
    follower.gate_blocked_for = lambda now: held_s


def test_a_gate_stall_is_named_rather_than_left_running(monkeypatch):
    """Two obstacle sources, one world each.

    safety_gate works on raw returns and cannot name anything; the follower
    works on classified clusters and cannot see what was never clustered.
    Something in only the first - a bush leaning in, a thin post, clutter
    filed as outside_band - left the follower reporting a clear corridor and
    commanding 0.80 m/s into a gate that zeroed every frame. blocked_since
    never started, because the follower had no threat to start it with, so
    the chair stood at wp 1218 reading DWA v=0.80 until someone walked over.
    """
    _module, follower, published, commanded = dwa_with([], monkeypatch)
    blocking_gate(follower)

    follower.step()

    assert published and published[0].startswith("HOLD:GATE_STALL")
    assert commanded == ["STOP"]


def test_the_stall_report_carries_the_gate_own_numbers(monkeypatch):
    """A hold that does not say what stopped it is the state this replaces."""
    _module, follower, published, _commanded = dwa_with([], monkeypatch)
    blocking_gate(follower, detail="OBSTACLE at 1.42 m, envelope 2.55 m")

    follower.step()

    assert "1.42" in published[0] and "2.55" in published[0]


def test_a_stall_stops_asking_the_planner_for_arcs(monkeypatch):
    """It used to keep solving and keep commanding, which is what made the
    deadlock invisible: the status line read like a chair that was driving."""
    _module, follower, _published, _commanded = dwa_with([], monkeypatch)
    blocking_gate(follower)

    follower.step()

    assert follower.planner.calls == []


def test_a_gate_that_has_only_just_blocked_is_not_a_stall(monkeypatch):
    """Ordinary vetoes come and go while the chair drives past things. Only
    a refusal that persists is evidence of a source disagreement."""
    _module, follower, published, _commanded = dwa_with([], monkeypatch)
    blocking_gate(follower, held_s=0.4)

    follower.step()

    assert not any(p.startswith("HOLD:GATE_STALL") for p in published)


def test_our_own_wait_still_wins_over_the_stall(monkeypatch):
    """When the cluster producer can see it, the threat rules belong to the
    threat rules. The stall branch is only for what they cannot see."""
    _module, follower, published, _commanded = dwa_with(
        [walking(1.0)], monkeypatch)
    blocking_gate(follower)

    follower.step()

    assert published == ["HOLD:DWA_WAIT"]


def test_the_stall_never_authorises_going_round(monkeypatch):
    """The 2026-08-05 lesson, kept as a test.

    A raw source has no identity, and the guard that let one authorise a
    bypass sent the chair at a wall. The gate can offer a distance and a
    side and nothing else, so what comes out of this branch is a stop and a
    message for a person - never a manoeuvre.
    """
    module, follower, published, commanded = dwa_with([], monkeypatch)
    blocking_gate(follower)
    went_round = []
    follower.take_a_way_round = lambda clear_for_m: went_round.append(clear_for_m)

    follower.step()

    assert went_round == []
    assert commanded == ["STOP"]
    assert all(p.startswith("HOLD:GATE_STALL") for p in published)


@pytest.mark.parametrize("reason", ["CLOUD_STALE", "INPUT_STALE",
                                    "INPUT_INVALID", "REVERSE", ""])
def test_only_obstacle_shaped_vetoes_read_as_something_in_the_road(reason):
    """Folding a dead sensor in here would relabel it as an object ahead,
    and those faults have their own handling."""
    module, _Stamp = load_follower("dwa_follower")
    assert not module.gate_stall(reason, 10.0)


@pytest.mark.parametrize("reason", ["OBSTACLE", "OBSTACLE_SWEEP"])
def test_both_obstacle_vetoes_count(reason):
    module, _Stamp = load_follower("dwa_follower")
    assert module.gate_stall(reason, 2.0)
    assert not module.gate_stall(reason, 0.1)
    assert not module.gate_stall(reason, None)
