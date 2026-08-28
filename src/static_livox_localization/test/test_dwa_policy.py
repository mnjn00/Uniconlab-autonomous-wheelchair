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
             last_yaw_rate=0.0, last_speed=None,
             obstacle_floor_m=None, candidate_veto=None):
        self.calls.append({"obstacles": list(obstacles),
                           "speed_cap": speed_cap,
                           "obstacle_floor_m": obstacle_floor_m,
                           "candidate_veto": candidate_veto})
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
    follower.person_memory = None
    follower.direct_person_threats = ()
    follower.person_static_track_id = None
    follower.person_static_since_s = None
    follower.person_static_last_stamp_s = None
    follower.person_bypass_committed_track_id = None
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


def summary_at(stamp, objects):
    return cg.parse_summary(json.dumps(
        {"stamp": stamp, "status": "OK", "objects": objects}))


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


def test_recorded_stationary_person_eventually_allows_safe_bypass(monkeypatch):
    """The 220341 bag held the same STATIC person for 28.2 seconds.

    DWA had hard-mask-contained arcs at every recorded waypoint checked, but
    the person label withheld their geometry from the planner forever. Ten
    seconds of direct same-track STATIC evidence must authorize DWA to test
    those arcs, without authorizing on the tracker's first STATIC frame.
    """
    person = walking(1.6)
    person.update({"id": 1641, "motion": ct.STATIC})
    _module, follower, published, commanded = dwa_with(
        [person], monkeypatch)

    for index in range(50):
        follower.cluster_summary = summary_at(
            100.0 + index * 0.2, [person])
        follower.step()

    assert follower.planner.calls == []
    assert published[-1] == "HOLD:DWA_WAIT"
    assert commanded[-1] == "STOP"

    follower.cluster_summary = summary_at(110.0, [person])
    follower.step()

    assert len(follower.planner.calls) == 1
    assert follower.planner.calls[0]["obstacles"], \
        "the recorded stationary person never reached DWA geometry planning"
    assert follower.planner.calls[0]["obstacle_floor_m"] == 0.50
    assert follower.planner.calls[0]["speed_cap"] <= 0.35


def test_committed_person_bypass_survives_lateral_arc(monkeypatch):
    # Given: the latest field-bag sequence: one directly observed static
    # person commits, then appears laterally outside the narrow stop corridor
    # while remaining inside the planner's wider maneuver corridor.
    person = walking(1.6)
    person.update({"id": 16, "motion": ct.STATIC})
    _module, follower, _published, _commanded = dwa_with(
        [person], monkeypatch)
    for index in range(51):
        follower.cluster_summary = summary_at(
            100.0 + index * 0.2, [person])
        follower.step()
    committed_calls = len(follower.planner.calls)
    person.update({
        "x": 1.12,
        "y": -0.75,
        "size": [0.28, 0.28, 0.86],
        "profile": {
            "bin_m": 0.2,
            "y0": -1.0,
            "min_x": [1.08, 0.98],
        },
    })

    # When: the same static track remains directly observed through the arc.
    for index in range(6):
        follower.cluster_summary = summary_at(
            110.2 + index * 0.2, [person])
        follower.step()

    # Then: every cycle remains a person-bypass plan instead of forgetting
    # the commitment and steering back toward the person.
    lateral_calls = follower.planner.calls[committed_calls:]
    assert len(lateral_calls) == 6
    assert all(call["obstacles"] for call in lateral_calls)
    assert all(
        call["obstacle_floor_m"] == 0.50
        for call in lateral_calls
    )


def test_stationary_person_is_watched_from_plan_ahead_before_bypass(
        monkeypatch):
    person = walking(4.0)
    person.update({"id": 1641, "motion": ct.STATIC})
    _module, follower, published, commanded = dwa_with(
        [person], monkeypatch, threat_distance_stop_radius=1.0)

    for index in range(50):
        follower.cluster_summary = summary_at(
            100.0 + index * 0.2, [person])
        follower.step()

    assert follower.planner.calls == []
    assert published[-1] == "HOLD:DWA_WAIT"
    assert commanded[-1] == "STOP"

    follower.cluster_summary = summary_at(110.0, [person])
    follower.step()

    assert len(follower.planner.calls) == 1
    assert follower.planner.calls[0]["obstacles"]


def test_one_static_frame_after_long_motion_does_not_authorize_bypass(
        monkeypatch):
    person = walking(1.6)
    person["id"] = 1641
    _module, follower, published, commanded = dwa_with(
        [person], monkeypatch)

    for index in range(50):
        follower.cluster_summary = summary_at(
            100.0 + index * 0.2, [person])
        follower.step()
    person["motion"] = ct.STATIC
    follower.cluster_summary = summary_at(110.0, [person])
    follower.step()

    assert follower.planner.calls == []
    assert published[-1] == "HOLD:DWA_WAIT"
    assert commanded[-1] == "STOP"


def test_a_person_dropout_restarts_static_bypass_qualification(monkeypatch):
    person = walking(1.6)
    person.update({"id": 1641, "motion": ct.STATIC})
    _module, follower, published, commanded = dwa_with(
        [person], monkeypatch)

    for index in range(50):
        follower.cluster_summary = summary_at(
            100.0 + index * 0.2, [person])
        follower.step()
    follower.cluster_summary = summary_at(110.0, [])
    follower.step()
    for index in range(50):
        follower.cluster_summary = summary_at(
            110.2 + index * 0.2, [person])
        follower.step()

    assert follower.planner.calls == []
    assert published[-1] == "HOLD:DWA_WAIT"
    assert commanded[-1] == "STOP"

    follower.cluster_summary = summary_at(120.2, [person])
    follower.step()

    assert len(follower.planner.calls) == 1


def test_a_producer_stamp_gap_restarts_static_bypass_qualification(
        monkeypatch):
    person = walking(1.6)
    person.update({"id": 1641, "motion": ct.STATIC})
    _module, follower, published, commanded = dwa_with(
        [person], monkeypatch)

    for index in range(50):
        follower.cluster_summary = summary_at(
            100.0 + index * 0.2, [person])
        follower.step()
    for index in range(50):
        follower.cluster_summary = summary_at(
            110.4 + index * 0.2, [person])
        follower.step()

    assert follower.planner.calls == []
    assert published[-1] == "HOLD:DWA_WAIT"
    assert commanded[-1] == "STOP"

    follower.cluster_summary = summary_at(120.4, [person])
    follower.step()

    assert len(follower.planner.calls) == 1


def test_a_replacement_person_id_restarts_bypass_qualification(monkeypatch):
    first = walking(1.6)
    first.update({"id": 1641, "motion": ct.STATIC})
    replacement = walking(1.6)
    replacement.update({"id": 1689, "motion": ct.STATIC})
    _module, follower, published, commanded = dwa_with(
        [first], monkeypatch)

    for index in range(50):
        follower.cluster_summary = summary_at(
            100.0 + index * 0.2, [first])
        follower.step()
    for index in range(50):
        follower.cluster_summary = summary_at(
            110.0 + index * 0.2, [replacement])
        follower.step()

    assert follower.planner.calls == []
    assert published[-1] == "HOLD:DWA_WAIT"
    assert commanded[-1] == "STOP"

    follower.cluster_summary = summary_at(120.0, [replacement])
    follower.step()

    assert len(follower.planner.calls) == 1


def test_a_moving_person_revokes_an_authorized_bypass(monkeypatch):
    person = walking(1.6)
    person.update({"id": 1641, "motion": ct.STATIC})
    _module, follower, published, commanded = dwa_with(
        [person], monkeypatch)

    for index in range(51):
        follower.cluster_summary = summary_at(
            100.0 + index * 0.2, [person])
        follower.step()
    assert len(follower.planner.calls) == 1

    person["motion"] = ct.MOVING
    follower.cluster_summary = summary_at(110.2, [person])
    follower.step()

    assert len(follower.planner.calls) == 1
    assert published[-1] == "HOLD:DWA_WAIT"
    assert commanded[-1] == "STOP"


def test_a_second_moving_person_prevents_static_person_bypass(monkeypatch):
    stationary = walking(1.6)
    stationary.update({"id": 1641, "motion": ct.STATIC})
    moving = walking(2.0, y=1.2)
    moving["id"] = 1642
    people = [stationary, moving]
    _module, follower, published, commanded = dwa_with(
        people, monkeypatch)

    for index in range(51):
        follower.cluster_summary = summary_at(
            100.0 + index * 0.2, people)
        follower.step()

    assert follower.planner.calls == []
    assert published[-1] == "HOLD:DWA_WAIT"
    assert commanded[-1] == "STOP"


def test_a_person_without_valid_track_identity_never_authorizes_bypass(
        monkeypatch):
    person = walking(1.6)
    person["motion"] = ct.STATIC
    _module, follower, published, commanded = dwa_with(
        [person], monkeypatch)

    for index in range(60):
        follower.cluster_summary = summary_at(
            100.0 + index * 0.2, [person])
        follower.step()

    assert follower.planner.calls == []
    assert published[-1] == "HOLD:DWA_WAIT"
    assert commanded[-1] == "STOP"


def test_a_person_with_malformed_geometry_never_authorizes_bypass(
        monkeypatch):
    person = walking(1.6)
    person.update({
        "id": 1641,
        "motion": ct.STATIC,
        "profile": {"bin_m": 0.2, "y0": -0.2, "min_x": ["broken"]},
    })
    _module, follower, published, commanded = dwa_with(
        [person], monkeypatch)

    for index in range(60):
        follower.cluster_summary = summary_at(
            100.0 + index * 0.2, [person])
        follower.step()

    assert follower.planner.calls == []
    assert published[-1] == "HOLD:DWA_WAIT"
    assert commanded[-1] == "STOP"


def test_a_person_gets_the_wider_055m_stop_corridor(monkeypatch):
    """A walking person's near flank at 0.50 m must still stop the chair.

    The ordinary obstacle corridor ends at 0.45 m.  The person-only stop
    corridor extends to 0.55 m without widening the geometry handed to DWA.
    """
    _module, follower, published, commanded = dwa_with(
        [walking(1.0, y=0.8)], monkeypatch)

    follower.step()

    assert published == ["HOLD:DWA_WAIT"]
    assert commanded == ["STOP"]
    assert follower.planner.calls == []


def test_a_person_stops_at_120_percent_of_the_dynamic_radius(monkeypatch):
    """A person 1.7 m ahead is inside the 1.8 m person-only radius."""
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


def test_person_edge_and_dropout_dither_stays_in_wait(monkeypatch):
    """Measured edge jitter and one missing frame must not restart DWA."""
    _module, follower, published, commanded = dwa_with(
        [walking(1.6, y=0.74)], monkeypatch,
        threat_distance_stop_radius=1.5)
    frames = (
        (100.0, [walking(1.6, y=0.74)]),
        (100.2, [walking(1.6, y=0.76)]),
        (100.5, []),
        (100.8, [walking(1.6, y=0.76)]),
        (101.0, [walking(1.6, y=0.74)]),
    )

    for stamp, objects in frames:
        follower.cluster_summary = summary_at(stamp, objects)
        published[:] = []
        commanded[:] = []
        planner_calls = len(follower.planner.calls)

        follower.step()

        assert published == ["HOLD:DWA_WAIT"], (stamp, published)
        assert commanded == ["STOP"], (stamp, commanded)
        assert len(follower.planner.calls) == planner_calls


def test_a_person_stop_survives_the_dynamic_radius_shrinking(monkeypatch):
    """Recorded failure: braking shrinks the next cycle's stopping envelope.

    One unchanged person at 1.10 m alternated DWA and WAIT as the person-
    scaled radius moved between 0.96 m and 1.20 m. Once the larger envelope
    has required a stop, a smaller stopped-chair envelope must not restart
    the planner while that same person remains at the boundary.
    """
    _module, follower, published, commanded = dwa_with(
        [walking(1.4)], monkeypatch)
    radii = iter((0.80, 1.00, 0.80, 0.80))
    follower.stop_radius = lambda: next(radii)
    outcomes = []

    for _ in range(4):
        published[:] = []
        commanded[:] = []
        planner_calls = len(follower.planner.calls)

        follower.step()

        outcomes.append((
            commanded == ["STOP"],
            len(follower.planner.calls) - planner_calls,
        ))

    assert outcomes == [
        (False, 1),
        (True, 0),
        (True, 0),
        (True, 0),
    ]


def test_a_latched_person_releases_after_moving_away(monkeypatch):
    _module, follower, published, commanded = dwa_with(
        [walking(1.4)], monkeypatch)
    radii = iter((1.00, 0.80))
    follower.stop_radius = lambda: next(radii)

    follower.step()

    assert commanded == ["STOP"]
    follower.cluster_summary = summary_at(100.2, [walking(2.0)])
    published[:] = []
    commanded[:] = []
    planner_calls = len(follower.planner.calls)

    follower.step()

    assert commanded != ["STOP"]
    assert len(follower.planner.calls) == planner_calls + 1


def test_a_person_survives_a_producer_dropout(monkeypatch):
    _module, follower, _published, _commanded = dwa_with(
        [walking(2.0)], monkeypatch)
    assert follower.corridor_threat(0.0) is not None

    follower.cluster_summary = summary_at(100.5, [])

    assert follower.corridor_threat(0.0) is not None


def test_a_person_who_really_leaves_is_let_go(monkeypatch):
    _module, follower, _published, _commanded = dwa_with(
        [walking(2.0)], monkeypatch)
    follower.corridor_threat(0.0)

    follower.cluster_summary = summary_at(101.5, [])

    assert follower.corridor_threat(0.0) is None


def test_only_a_person_is_held(monkeypatch):
    _module, follower, _published, _commanded = dwa_with(
        [parked(2.0)], monkeypatch)
    assert follower.corridor_threat(0.0) is not None

    follower.cluster_summary = summary_at(100.2, [])

    assert follower.corridor_threat(0.0) is None


def test_the_memory_is_timed_off_the_producer_clock(monkeypatch):
    _module, follower, _published, _commanded = dwa_with(
        [walking(2.0)], monkeypatch)
    follower.corridor_threat(0.0)

    follower.cluster_summary = summary_at(100.1, [])
    assert follower.corridor_threat(0.0) is not None
    follower.cluster_summary = summary_at(140.0, [])
    assert follower.corridor_threat(0.0) is None


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
    to wait for at full speed.  The near fixture stays just outside the
    person-only stop radius so this test measures slowing, not stopping."""
    _module, near, _p, _c = dwa_with([walking(2.2)], monkeypatch)
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
        assert "self.stop_radius_for(threat)" in text, \
            "%s must apply the shared person-aware stop radius" % name
        assert "self.threat_blocks(threat," in text, \
            "%s must apply the shared person-stop hysteresis" % name
        if name == "dwa_follower":
            assert "decision == WAIT" in text
            assert "GO_ROUND, PERSON_BYPASS" in text
        else:
            assert "decision in (WAIT, PERSON_BYPASS)" in text
        assert "avoidance_decision(" not in text, \
            "%s must not re-implement the decision" % name


# ------------------------------------------- the gate refusing what we cannot see

def blocking_gate(follower, reason="OBSTACLE", held_s=2.0,
                  detail="OBSTACLE at 1.6 m, envelope 2.6 m"):
    follower.gate_reason = reason
    follower.gate_detail = detail
    follower.gate_blocked_for = lambda now: held_s


def test_person_bypass_remembers_the_yaw_rejected_by_the_raw_gate():
    module, _Stamp = load_follower("person_bypass_dwa_follower")
    follower = module.PersonBypassDwaFollower.__new__(
        module.PersonBypassDwaFollower)
    follower.gate_reason = ""
    follower.gate_blocked_since = None
    follower.gate_detail = ""
    follower._gate_rejected_yaw_rates = set()
    message = types.SimpleNamespace(data=json.dumps({
        "reason": "OBSTACLE",
        "trajectory_override_reason": "REQUESTED_PATH_COLLISION",
        "trajectory_requested_w": 0.5,
    }))

    follower.on_gate_status(message)

    assert follower._gate_rejected_yaw_rates == {0.5}


def test_static_non_person_threat_publishes_a_trajectory_permit(monkeypatch):
    module, Stamp = load_follower("person_bypass_dwa_follower")
    follower = module.PersonBypassDwaFollower.__new__(
        module.PersonBypassDwaFollower)
    published = []
    follower.qualifier = types.SimpleNamespace(
        reset=lambda: None,
        inactive=lambda now_s, reason: types.SimpleNamespace(
            active=False, reason=reason))
    follower.observed_person_permit = lambda now: types.SimpleNamespace(
        active=False, reason="NO_PERSON")
    follower.publish_permit = lambda permit: published.append(permit)
    follower.person_bypass_permit_lifetime_s = 0.45
    follower.person_bypass_maximum_gap_s = 0.45
    follower.person_bypass_speed_mps = 0.35
    follower.person_bypass_clearance_m = 0.80
    follower._gate_rejected_yaw_rates = set()
    follower._gate_rejected_track_id = None
    follower.planner = types.SimpleNamespace(max_speed=0.8)
    threat = types.SimpleNamespace(
        is_person=False,
        parked=True,
        track_id=44,
        observed_stamp_s=99.8,
        distance_m=2.0,
        lateral_m=0.3,
        directly_observed=True,
        geometry_valid=True,
        motion="static")
    monkeypatch.setattr(
        module.DwaFollower, "avoidance_for",
        lambda self, now, observed, blocking: module.GO_ROUND)

    decision = follower.avoidance_for(Stamp(100.0), threat, True)

    assert decision == module.GO_ROUND
    assert published[-1].active
    assert published[-1].reason == "STATIC_OBJECT_BYPASS"


def bypass_fixture(monkeypatch):
    module, Stamp = load_follower("person_bypass_dwa_follower")
    follower = module.PersonBypassDwaFollower.__new__(module.PersonBypassDwaFollower)
    follower.qualifier = module.StaticPersonQualifier()
    follower.person_bypass_maximum_forward_m = 8.0
    follower.person_bypass_maximum_lateral_m = 1.0
    follower.person_bypass_lateral_hysteresis_m = 0.25
    follower.tracking_state = "TRACKING"
    follower._gate_rejected_yaw_rates = set()
    follower._gate_rejected_track_id = None
    follower.active_trajectory_permit = None
    follower.planner = types.SimpleNamespace(max_speed=0.8)
    permits = []
    def publish(permit):
        permits.append(permit)
        follower._permit_published_this_cycle = True
    follower.publish_permit = publish
    # Keep the real base decision in other tests. Here the ordinary query
    # result is varied explicitly to verify it cannot reset person evidence.
    monkeypatch.setattr(module.DwaFollower, "avoidance_for",
                        lambda *a: cg.CLEAR)
    monkeypatch.setattr(module.dwa_core, "OBSTACLE_FLOOR_M", .5)
    return module, Stamp, follower, permits


def recorded_side_person():
    # 13:35:50, track 5584, summary recorded during the ~21 s PERSON hold.
    return dict(id=5584, **{"class": "person"}, motion="static",
                source="geometric", x=1.339, y=.817, size=[.52, .43, .88],
                points=1761, profile=dict(bin_m=.2, min_x=[1.13,1.08,1.17], y0=.6))


def qualify_bypass_fixture(follower, Stamp, people=None, start=10.0):
    people = people if people is not None else [recorded_side_person()]
    for i in range(17):
        now = start + .2*i
        follower.cluster_summary = summary_at(now, people)
        permit = follower.observed_person_permit(Stamp(now))
    return permit


def test_recorded_semantic_only_side_person_qualifies_without_nearest_query(monkeypatch):
    module, Stamp, follower, permits = bypass_fixture(monkeypatch)
    item = recorded_side_person()
    summary = summary_at(10., [item])
    assert cg.nearest_threat(summary, .55, labels=("person",)) is None
    assert cg.nearest_threat(summary, .65, labels=("person",)).track_id == 5584
    follower.corridor_threat = lambda *a: pytest.fail("narrow query must not gate qualification")
    permit = qualify_bypass_fixture(follower, Stamp)
    assert permit.active and permit.static_for_s >= 3.
    assert follower.avoidance_for(Stamp(13.2), None, False) == module.GO_ROUND
    assert permits[-1].track_id == 5584
    assert follower.active_trajectory_permit is not None
    assert follower.planner.max_speed == .35


@pytest.mark.parametrize('ordinary', [cg.CLEAR, cg.GO_ROUND, cg.WAIT])
def test_nearer_static_object_does_not_erase_person_timer(monkeypatch, ordinary):
    module, Stamp, follower, permits = bypass_fixture(monkeypatch)
    monkeypatch.setattr(module.DwaFollower, "avoidance_for", lambda *a: ordinary)
    nearer = dict(id=5748, **{"class": "obstacle"}, motion="static",
                  x=1.0, y=0., size=[.1,.1,.2], points=25)
    threat = types.SimpleNamespace(is_person=False, parked=True)
    for i in range(18):
        now = 10.0 + .2*i
        follower.cluster_summary = summary_at(now, [nearer, recorded_side_person()])
        result = follower.avoidance_for(Stamp(now), threat, True)
        if i < 15:
            assert result == module.WAIT
            assert not permits[-1].active
    assert permits[-1].active and permits[-1].track_id == 5584
    assert permits[-1].static_for_s >= 3.
    assert permits[-1].reason == "STATIC_PERSON_BYPASS"
    assert result == (module.WAIT if ordinary == cg.WAIT else module.GO_ROUND)


@pytest.mark.parametrize('motion', ['moving', 'unknown'])
@pytest.mark.parametrize('ordinary', [cg.CLEAR, cg.WAIT, cg.GO_ROUND])
def test_person_permit_never_overrides_another_nonstatic_obstacle(monkeypatch, motion, ordinary):
    module, Stamp, follower, permits = bypass_fixture(monkeypatch)
    qualify_bypass_fixture(follower, Stamp)
    monkeypatch.setattr(module.DwaFollower, "avoidance_for", lambda *a: ordinary)
    threat = types.SimpleNamespace(is_person=False, parked=False, motion=motion)
    assert follower.avoidance_for(Stamp(13.2), threat, True) == module.WAIT
    assert permits[-1].active  # evidence retained, motion still withheld
    assert follower.active_trajectory_permit is None


@pytest.mark.parametrize('change,reason', [
    ('moving', 'PERSON_NOT_CONFIRMED_STATIC'),
    ('unknown', 'PERSON_NOT_CONFIRMED_STATIC'),
    ('missing', 'NO_PERSON'), ('multiple', 'MULTIPLE_PEOPLE'),
    ('id', 'QUALIFYING_STATIC_PERSON'), ('jump', 'QUALIFYING_STATIC_PERSON'),
    ('stale', 'PERSON_OBSERVATION_STALE'),
    ('localization', 'LOCALIZATION_NOT_TRACKING'),
    ('close', 'PERSON_TOO_CLOSE'), ('learned', 'PERSON_NOT_CONFIRMED_STATIC')])
def test_person_evidence_changes_still_revoke_permission(monkeypatch, change, reason):
    module, Stamp, follower, permits = bypass_fixture(monkeypatch)
    assert qualify_bypass_fixture(follower, Stamp).active
    item = recorded_side_person()
    people = [item]
    stamp, now = 13.4, 13.4
    if change in ('moving','unknown'): item['motion'] = change
    elif change == 'missing': people = []
    elif change == 'multiple': people.append(dict(item, id=5752))
    elif change == 'id': item['id'] = 5757
    elif change == 'jump': item['x'] += .5
    elif change == 'stale': stamp = 12.0
    elif change == 'localization': follower.tracking_state = 'LOST'
    elif change == 'close': item['x'] = .7
    elif change == 'learned': item['source'] = 'learned_only'
    follower.cluster_summary = summary_at(stamp, people)
    permit = follower.observed_person_permit(Stamp(now))
    assert not permit.active and permit.reason == reason
    # Even the base's separate remembered-person readiness cannot release.
    monkeypatch.setattr(module.DwaFollower, 'avoidance_for', lambda *a: cg.PERSON_BYPASS)
    assert follower.avoidance_for(Stamp(now), types.SimpleNamespace(is_person=True), True) == module.WAIT


def test_semantic_validation_retains_same_track_in_qualifier_hysteresis(monkeypatch):
    module, Stamp, follower, _permits = bypass_fixture(monkeypatch)
    person = dict(recorded_side_person(), x=3., y=1., size=[.7,.7,1.7])
    assert qualify_bypass_fixture(follower, Stamp, [person]).active
    follower.cluster_summary = summary_at(13.3, [dict(person, y=1.2)])
    assert follower.observed_person_permit(Stamp(13.3)).active
    person = dict(person, y=1.413)  # box edge 1.063: inside retention, not acquisition
    follower.cluster_summary = summary_at(13.4, [person])
    permit = follower.observed_person_permit(Stamp(13.4))
    assert permit.active
    semantic, _ = load_follower('person_bypass_semantic_supervisor')
    supervisor = semantic.PersonBypassSemanticSupervisor.__new__(semantic.PersonBypassSemanticSupervisor)
    supervisor.bypass_permit = permit
    supervisor.summary = follower.cluster_summary
    supervisor.maximum_permit_age_s = .45
    supervisor.maximum_target_error_m = .45
    supervisor.bypass_maximum_forward_m = 8.
    supervisor.bypass_maximum_lateral_m = 1.
    supervisor.bypass_lateral_hysteresis_m = .25
    assert supervisor.validated_bypass(13.4)[0] is not None
    supervisor.summary = summary_at(13.4, [person, dict(person,id=999)])
    assert supervisor.validated_bypass(13.4) == (None,None)
    supervisor.summary = summary_at(13.4, [dict(person,motion='moving')])
    assert supervisor.validated_bypass(13.4) == (None,None)


def test_person_qualification_while_paused_does_not_send_motion(monkeypatch):
    module, Stamp, follower, permits = bypass_fixture(monkeypatch)
    # The real wrapper step must publish qualification even when the base
    # hold ladder returns immediately, but must not activate a trajectory.
    monkeypatch.setattr(module.DwaFollower, 'step', lambda self: None)
    for i in range(18):
        now = 10. + .2*i
        monkeypatch.setattr(module.rospy.Time, 'now', lambda: Stamp(now))
        follower.cluster_summary = summary_at(now, [recorded_side_person()])
        follower.step()
    assert any(p.active and p.track_id == 5584 for p in permits)
    assert len(permits) == 18 and permits[-1].active
    assert follower.active_trajectory_permit is None
    assert follower.planner.max_speed == .8


def test_semantic_only_person_reaches_dwa_with_geometry_and_precheck(monkeypatch):
    _base_module, template, published, commanded = dwa_with([], monkeypatch)
    module, Stamp, follower, _permits = bypass_fixture(monkeypatch)
    follower.__dict__.update(template.__dict__)
    follower.planner.max_speed = .8
    reference_module = module.DwaFollower.step.__globals__['mpc_speed']
    monkeypatch.setattr(reference_module, 'shaped_reference',
                        lambda *a, **k: (np.array([.6,.6]), None))
    assert qualify_bypass_fixture(follower, Stamp, start=96.8).active
    monkeypatch.setattr(module.rospy.Time, 'now', lambda: Stamp(100.))
    veto_calls = []
    def candidate_veto(now, decision, command):
        assert decision == module.GO_ROUND
        assert follower.active_trajectory_permit.track_id == 5584
        assert follower.planner.max_speed == .35
        veto_calls.append(True)
        return lambda v,w: False
    follower.planner_candidate_veto = candidate_veto
    follower.step()
    assert veto_calls and len(follower.planner.calls) == 1
    plan = follower.planner.calls[0]
    assert plan['obstacles'] and callable(plan['candidate_veto'])
    assert plan['speed_cap'] <= .6  # actual solver also enforces planner.max_speed
    assert commanded and commanded[0] != 'STOP'
    assert follower.planner.max_speed == .8  # temporary cap restored


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
