"""Switching the safety policies off must not switch the failsafe off.

The mode exists so one run can answer one question - does localization stay
attached over the whole 0727 line - without a band refusal or an obstacle
stop ending the measurement first, since from outside a stationary chair
they all look the same. The operator's joystick is what replaces the guards.

That makes exactly one thing non-negotiable: everything the joystick
override rests on has to keep binding. MANUAL_MODE is the override itself,
and BASE_STALE is whether the channel that reports it is still alive - a
build that drops BASE_STALE does not have a reduced failsafe, it has none,
because the chair would drive on with no way to observe that the operator
had taken control. The tests below are mostly about that one sentence.
"""

import importlib.util
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    spec = importlib.util.spec_from_file_location(
        name, SCRIPTS / ("%s.py" % name))
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


dp = load("drive_policy")


def test_with_policies_on_it_is_just_first_reason_wins():
    """The enabled path has to reduce to the chain it replaced, or every
    field hour behind that chain stops counting for anything."""
    candidates = [("NO_CLOUD", dp.POLICY),
                  ("LOCALIZATION_LOST", dp.POLICY),
                  ("MANUAL_MODE", dp.OVERRIDE)]
    assert dp.evaluate_holds(iter(candidates), True) == ("NO_CLOUD", None)


def test_the_joystick_still_stops_the_chair_with_the_policies_off():
    candidates = [("OFF_BAND", dp.POLICY), ("MANUAL_MODE", dp.OVERRIDE)]
    binding, _ = dp.evaluate_holds(iter(candidates), False)
    assert binding == "MANUAL_MODE"


def test_a_dead_wheel_link_still_stops_the_chair_with_the_policies_off():
    """BASE_STALE is not a policy: /wheel_status is how MANUAL_MODE is
    observed at all, so driving through it means driving with the override
    disconnected."""
    candidates = [("LOCALIZATION_LOST", dp.POLICY),
                  ("BASE_STALE", dp.OVERRIDE)]
    binding, _ = dp.evaluate_holds(iter(candidates), False)
    assert binding == "BASE_STALE"


@pytest.mark.parametrize("reason", [
    "OFF_BAND", "OFF_ROUTE", "NO_CLOUD", "LOCALIZATION_LOST",
    "LOCALIZATION_DEGRADED_TIMEOUT", "ODOM_STALE"])
def test_a_lone_policy_does_not_stop_the_chair_but_is_reported(reason):
    binding, suppressed = dp.evaluate_holds(iter([(reason, dp.POLICY)]), False)
    assert binding is None
    assert suppressed == reason


def test_the_first_suppressed_policy_is_the_one_reported():
    """Priority order survives the mode: what would have bitten first is
    what the run needs to know about."""
    candidates = [("LOCALIZATION_LOST", dp.POLICY), ("OFF_BAND", dp.POLICY)]
    assert dp.evaluate_holds(iter(candidates), False) == (
        None, "LOCALIZATION_LOST")


def test_candidates_after_a_binding_hold_are_never_evaluated():
    """The chain is consumed lazily on purpose: NO_POSE is what guarantees
    the position tests under it have a position to read, so a generator that
    ran past it would dereference None on the vehicle."""
    reached = []

    def candidates():
        yield "NO_POSE", dp.OVERRIDE
        reached.append("evaluated the position tests without a position")
        yield "OFF_BAND", dp.POLICY

    assert dp.evaluate_holds(candidates(), False)[0] == "NO_POSE"
    assert reached == []


def test_the_disabled_announcement_names_what_it_dropped():
    """"Diagnostic mode" tells an operator nothing. Someone has to be able
    to read the log line and know the chair will not stop for a bollard."""
    line = dp.announce(False, "waypoint_follower", ["obstacles", "the band"])
    assert "obstacles" in line and "the band" in line
    assert "joystick" in line.lower()


# --------------------------------------------------------------- the wiring
# evaluate_holds is only correct if the nodes tag their candidates correctly,
# and a mistagged override is the failure this whole mode has to not have.

def follower_text():
    return (SCRIPTS / "waypoint_follower.py").read_text(encoding="utf-8")


@pytest.mark.parametrize("reason", [
    '("DONE" if self.done else "PAUSED")', '"NO_POSE"', '"BASE_STALE"',
    '"MANUAL_MODE"'])
def test_the_follower_tags_its_overrides_as_overrides(reason):
    assert "yield %s, OVERRIDE" % reason in follower_text()


@pytest.mark.parametrize("reason", ['"NO_CLOUD"', '"OFF_ROUTE"', '"OFF_BAND"'])
def test_the_follower_tags_its_judgements_as_policy(reason):
    assert "yield %s, POLICY" % reason in follower_text()


def test_the_follower_defaults_to_guarded():
    """An absent parameter is a normal run. The dangerous state has to be
    the one someone typed."""
    assert 'rospy.get_param("~safety_policies", True)' in follower_text()
    assert 'rospy.get_param("~safety_policies", True)' in \
        (SCRIPTS / "safety_gate.py").read_text(encoding="utf-8")


def test_the_gate_keeps_its_chain_integrity_checks():
    """INPUT_STALE and INPUT_INVALID are how the gate notices the planner
    died; REVERSE is unsensed at every setting. None of the three is a
    judgement about the world, so none may be behind the switch."""
    text = (SCRIPTS / "safety_gate.py").read_text(encoding="utf-8")
    for guard in ('reason = "INPUT_STALE"', 'reason = "INPUT_INVALID"',
                  'reason = "REVERSE"'):
        line = text[text.index(guard) - 200:text.index(guard)]
        assert "self.policies" not in line.split("elif")[-1]


def test_obstacle_detection_does_not_switch_off_with_the_policies():
    """The raw scan check used to be the half of obstacle detection that
    ~safety_policies could switch off; the cluster path was deliberately
    left outside that switch so something kept watching for people. With
    the raw check removed there is one source left, and it must stay
    outside the switch - otherwise policies:=false drives blind.
    """
    text = follower_text()
    start = text.index("def corridor_threat")
    body = text[start:text.index("\n    def ", start + 1)]
    assert "self.policies" not in body
    assert "self.cluster_threat(lateral_shift)" in body


# ------------------------------------------------- the real chain, end to end
# The tests above pin the tags by reading the source, which catches a
# mistagged yield but not a chain that never reaches it. These run the actual
# generator against the actual resolver.

class Stamp:
    """Enough of rospy.Time for the age comparisons in hold_candidates."""

    def __init__(self, seconds):
        self.seconds = float(seconds)

    def __sub__(self, other):
        return Stamp(self.seconds - other.seconds)

    def to_sec(self):
        return self.seconds


class Band:
    def __init__(self, inside=False):
        self.inside = inside

    def contains(self, xy, grace=0.0):
        return self.inside

    def lateral_limits(self, xy):
        # bypass_room_each_side asks the band how much room each side has.
        # The double answers with a corridor wide enough that the room test
        # never decides anything - these tests are about which side the
        # policy picks, not about how much band there is to pick it in. Room
        # has to clear BYPASS_OFFSET_MAX_M + BYPASS_EDGE_KEEP_M or the widest
        # rungs of the ladder disappear before the policy sees them.
        return 0.0, -3.0, 3.0


def follower_at(policies, drive_mode=65, wheel_age_s=0.0):
    """A follower positioned on its route, fully healthy, and off band."""
    import numpy as np
    from test_waypoint_follower_geometry import load_follower_module

    MotionEstimate = load("motion_safety").MotionEstimate
    module = load_follower_module()
    follower = module.WaypointFollower.__new__(module.WaypointFollower)
    follower.enabled = True
    follower.done = False
    follower.pose_xy = np.array([0.0, 0.0])
    follower.pose_stamp = Stamp(100.0)
    follower.cloud_stamp = Stamp(100.0)
    follower.motion = MotionEstimate(True, 100.0, 100.0, 0.4, 0.0, "")
    follower.degraded_since = None
    follower.tracking_state = "TRACKING"
    follower.tracking_reason = "OK"
    follower.reacquire_origin = None
    follower.wheel_status_stamp = Stamp(100.0 - wheel_age_s)
    follower.drive_mode = drive_mode
    follower.route_locked = True
    follower.waypoints = np.array([[0.0, 0.0], [1.0, 0.0]])
    follower.band = Band(inside=False)
    follower.policies = policies
    # Off for these: the cluster guard adds its own hold, and what is being
    # pinned here is the policy split, not the guard.
    follower.clusters_enabled = False
    follower.cluster_summary = None
    return module, follower


def resolve(policies, **kwargs):
    module, follower = follower_at(policies, **kwargs)
    return dp.evaluate_holds(
        follower.hold_candidates(Stamp(100.0)), follower.policies)


def test_off_band_stops_a_guarded_chair_and_not_an_unguarded_one():
    assert resolve(True) == ("OFF_BAND", None)
    assert resolve(False) == (None, "OFF_BAND")


def test_the_joystick_reaches_the_override_past_a_suppressed_policy():
    """The regression this mode could most easily introduce: OFF_BAND is
    evaluated before MANUAL_MODE is even reached in the old chain, so a
    resolver that stopped scanning at the first applicable candidate would
    drive straight through the operator taking control."""
    binding, _ = resolve(False, drive_mode=77)
    assert binding == "MANUAL_MODE"


def test_a_silent_wheel_link_reaches_the_override_past_a_suppressed_policy():
    binding, _ = resolve(False, wheel_age_s=5.0)
    assert binding == "BASE_STALE"


def test_a_dead_cluster_producer_stops_the_chair_with_the_policies_off():
    """Once the policies are off the cluster guard is the only thing still
    watching for people, so a producer that died silently would leave the
    chair driving on an empty object list - which reads exactly like clear
    road. Tagged OVERRIDE for the same reason BASE_STALE is."""
    _module, follower = follower_at(False)
    follower.clusters_enabled = True
    follower.cluster_summary = None

    binding, _ = dp.evaluate_holds(
        follower.hold_candidates(Stamp(100.0)), False)
    assert binding == "CLUSTERS_STALE"


def test_a_live_cluster_producer_does_not_hold_the_chair():
    _module, follower = follower_at(False)
    follower.clusters_enabled = True
    follower.cluster_summary = load("cluster_guard").parse_summary(
        '{"stamp": 99.8, "status": "OK", "objects": []}')

    binding, suppressed = dp.evaluate_holds(
        follower.hold_candidates(Stamp(100.0)), False)
    assert binding is None
    assert suppressed == "OFF_BAND"


# ----------------------------------------- the guarded path did not change
# Splitting one if/elif chain into a tagged generator plus a resolver is the
# kind of edit that looks equivalent and quietly is not: the old chain
# short-circuited at NO_CLOUD before MANUAL_MODE was ever reached, so the
# priority order is load-bearing. Everything behind the guarded path was
# validated in the field on 7/29 and none of it may move.

def old_chain(follower, now, F):
    """The decision as it stood at 639f2a4, verbatim."""
    import numpy as np
    reason = None
    if not follower.enabled or follower.done:
        reason = "DONE" if follower.done else "PAUSED"
    elif follower.pose_xy is None or \
            (now - follower.pose_stamp).to_sec() > F.POSE_STALE_S:
        reason = "NO_POSE"
    elif (now - follower.cloud_stamp).to_sec() > 1.0:
        reason = "NO_CLOUD"
    else:
        reason = F.motion_hold_reason(
            follower.motion, now.to_sec(), F.ODOM_STALE_S)
        if not reason:
            age = None if follower.degraded_since is None else \
                (now - follower.degraded_since).to_sec()
            reason = F.localization_hold_reason(
                follower.tracking_state, age, F.DEGRADED_STOP_S,
                reason=follower.tracking_reason, reacquire_m=None)
    if reason is None and \
            (now - follower.wheel_status_stamp).to_sec() > F.BASE_STALE_S:
        reason = "BASE_STALE"
    elif reason is None and follower.drive_mode is not None and \
            follower.drive_mode != F.AUTO_MODE:
        reason = "MANUAL_MODE"
    elif reason is None and follower.route_locked and np.min(np.linalg.norm(
            follower.waypoints - follower.pose_xy, axis=1)) > F.GEOFENCE_M:
        reason = "OFF_ROUTE"
    elif reason is None and follower.route_locked and \
            not follower.band.contains(
                follower.pose_xy, grace=F.BAND_RECOVER_MAX):
        reason = "OFF_BAND"
    return reason or None


def test_with_the_policies_on_every_reachable_state_decides_as_it_used_to():
    import itertools
    import numpy as np
    from test_waypoint_follower_geometry import load_follower_module

    module = load_follower_module()
    estimate = load("motion_safety").MotionEstimate
    now = 100.0
    axes = {
        "enabled": [True, False],
        "done": [True, False],
        "pose_age": [0.0, 5.0],
        "pose_none": [True, False],
        "cloud_age": [0.0, 5.0],
        "motion": [estimate(True, now, now, 0.4, 0.0, ""),
                   estimate(False, now, now, 0.0, 0.0, "ODOM_INVALID"),
                   estimate(True, now - 9, now - 9, 0.4, 0.0, "")],
        "track": ["TRACKING", "LOST", "DEGRADED", "ALIGNING"],
        "degraded_age": [None, 0.5, 9.0],
        "wheel_age": [0.0, 9.0],
        "mode": [65, 77, None],
        "locked": [True, False],
        "far": [True, False],
        "inband": [True, False],
    }
    names = list(axes)
    checked = 0
    for combo in itertools.product(*(axes[n] for n in names)):
        v = dict(zip(names, combo))
        follower = module.WaypointFollower.__new__(module.WaypointFollower)
        follower.enabled, follower.done = v["enabled"], v["done"]
        follower.pose_xy = None if v["pose_none"] else np.array([0.0, 0.0])
        follower.pose_stamp = Stamp(now - v["pose_age"])
        follower.cloud_stamp = Stamp(now - v["cloud_age"])
        follower.motion = v["motion"]
        follower.tracking_state = v["track"]
        follower.tracking_reason = "OK"
        follower.reacquire_origin = None
        follower.degraded_since = None if v["degraded_age"] is None else \
            Stamp(now - v["degraded_age"])
        follower.wheel_status_stamp = Stamp(now - v["wheel_age"])
        follower.drive_mode = v["mode"]
        follower.route_locked = v["locked"]
        follower.waypoints = np.array([[99.0, 99.0]]) if v["far"] else \
            np.array([[0.0, 0.0], [1.0, 0.0]])
        follower.band = Band(inside=v["inband"])
        follower.policies = True
        follower.clusters_enabled = False
        follower.cluster_summary = None

        binding, suppressed = dp.evaluate_holds(
            follower.hold_candidates(Stamp(now)), True)
        assert binding == old_chain(follower, Stamp(now), module), v
        assert suppressed is None, v
        checked += 1
    assert checked == 55296
