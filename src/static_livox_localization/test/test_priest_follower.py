"""The PRIEST follower's pure logic, and the contract it must not break.

The node is a drop-in alternative to waypoint_follower behind the same
topics, service and status strings - go.sh pings /waypoint_follower, the
black box records /waypoint_follower/status, stop.sh calls the same
service. A planner that arrived under different names would silently fall
outside every one of those, so the contract is pinned here as hard as the
geometry.

The geometry that matters most is the obstacle transform: objects arrive in
the lidar frame and the planner works in the map frame, and a frame slip
here does not crash - it plans around obstacles that are somewhere else.
"""

import importlib.util
import json
import sys
import types
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT / "scripts"


def load(name):
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
bf = load("body_frame")


def load_node():
    """priest_follower with ROS replaced by the little of it imported."""
    rospy = types.ModuleType("rospy")
    for name in ("loginfo", "logwarn", "logwarn_throttle", "logerr"):
        setattr(rospy, name, lambda *a, **k: None)
    modules = {"rospy": rospy}
    for package, names in {
        "diagnostic_msgs.msg": ["DiagnosticArray"],
        "geometry_msgs.msg": ["PoseWithCovarianceStamped", "Twist"],
        "nav_msgs.msg": ["Odometry"],
        "std_msgs.msg": ["Int16MultiArray", "String"],
        "std_srvs.srv": ["SetBool", "SetBoolResponse"],
    }.items():
        module = types.ModuleType(package)
        for name in names:
            setattr(module, name, type(name, (), {}))
        modules[package] = module
        root = package.split(".")[0]
        modules.setdefault(root, types.ModuleType(root))
        setattr(modules[root], package.split(".")[1], module)
    transformations = types.ModuleType("tf.transformations")
    transformations.quaternion_matrix = lambda value: np.eye(4)
    tf = types.ModuleType("tf")
    tf.transformations = transformations
    modules["tf"] = tf
    modules["tf.transformations"] = transformations

    saved = {name: sys.modules.get(name) for name in modules}
    sys.modules.update(modules)
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            "priest_follower_test", SCRIPTS / "priest_follower.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))
        for name, prior in saved.items():
            if prior is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = prior


pf = load_node()


def summary_of(objects):
    return cg.parse_summary(json.dumps(
        {"stamp": 100.0, "status": "OK", "objects": objects}))


def obj(x, y, motion, size=(0.4, 0.6, 1.2)):
    return {"class": "obstacle", "x": x, "y": y, "size": list(size),
            "points": 40, "motion": motion}


# --------------------------------------------------------- obstacle transform

def pose_at(x, y, yaw):
    T = np.eye(4)
    T[0, 0] = np.cos(yaw); T[0, 1] = -np.sin(yaw)
    T[1, 0] = np.sin(yaw); T[1, 1] = np.cos(yaw)
    T[:3, 3] = (x, y, 0.0)
    return T


def test_obstacles_land_where_the_map_says_they_are():
    """Chair at (10, 5) facing +y: an object 2 m ahead in the lidar frame
    must come out near (10, 7) in the map, not near (12, 5). The residual
    difference from the exact figure is the lidar-to-body extrinsic, which
    is a few centimetres - a frame slip is metres."""
    lidar_in_body, rotation = bf.lidar_extrinsics("builtin")
    circles, dropped = pf.planner_obstacles(
        [obj(2.0, 0.0, "static")], pose_at(10.0, 5.0, np.pi / 2),
        lidar_in_body, rotation)

    assert dropped == 0
    assert len(circles) == 1
    x, y, radius = circles[0]
    assert abs(x - 10.0) < 0.2
    assert abs(y - 7.0) < 0.2
    # circumscribed circle of a 0.4 x 0.6 footprint
    assert radius == pytest.approx(np.hypot(0.2, 0.3))


def test_moving_objects_are_not_planned_around():
    """A trajectory around where a walker is now is a trajectory into where
    they are next. They are handled by waiting, not by routing."""
    lidar_in_body, rotation = bf.lidar_extrinsics("builtin")
    circles, _ = pf.planner_obstacles(
        [obj(2.0, 0.0, "moving"), obj(4.0, 1.0, "static"),
         obj(5.0, -1.0, "unknown")],
        pose_at(0.0, 0.0, 0.0), lidar_in_body, rotation)

    assert len(circles) == 2  # static and unknown; moving excluded


def test_the_nearest_obstacles_survive_the_cap():
    lidar_in_body, rotation = bf.lidar_extrinsics("builtin")
    far_first = [obj(9.0 - i, 0.5, "static") for i in range(6)]
    circles, dropped = pf.planner_obstacles(
        far_first, pose_at(0.0, 0.0, 0.0), lidar_in_body, rotation, limit=3)

    assert len(circles) == 3
    assert dropped == 3
    ranges = [np.hypot(c[0], c[1]) for c in circles]
    assert ranges == sorted(ranges)


def test_a_malformed_object_is_not_given_an_invented_position():
    """It is skipped from the planner list - nearest_threat already reports
    it as blocking at zero range, which holds the chair without this list
    placing it somewhere it never was."""
    lidar_in_body, rotation = bf.lidar_extrinsics("builtin")
    circles, _ = pf.planner_obstacles(
        [{"class": "obstacle", "x": "near", "y": 0.0, "size": [1, 1, 1]}],
        pose_at(0.0, 0.0, 0.0), lidar_in_body, rotation)

    assert circles == []


# ------------------------------------------------------------------- waiting

def test_something_moving_close_ahead_is_waited_for():
    assert pf.wait_reason(summary_of([obj(2.0, 0.0, "moving")])) \
        == "OBSTACLE_WAIT"


def test_something_parked_close_ahead_is_not_waited_for():
    """Parked things are the planner's job - it routes around them."""
    assert pf.wait_reason(summary_of([obj(2.0, 0.0, "static")])) is None


def test_something_moving_far_ahead_is_not_waited_for_yet():
    assert pf.wait_reason(summary_of([obj(6.0, 0.0, "moving")])) is None


def test_a_producer_that_cannot_see_reads_as_someone_on_the_bumper():
    unusable = cg.parse_summary(json.dumps(
        {"stamp": 100.0, "status": "NO_CLOUD", "objects": []}))
    assert pf.wait_reason(unusable) == "OBSTACLE_WAIT"


# ------------------------------------------------------------ no diagnostics

def test_the_guards_cannot_be_switched_off():
    """The route follower's diagnostic switch exists to measure one thing on
    a path a person demonstrably drove. An unvalidated planner with guards
    off is an unsupervised experiment; the node refuses at startup."""
    with pytest.raises(ValueError):
        pf.require_guards(False)
    pf.require_guards(True)

    source = (SCRIPTS / "priest_follower.py").read_text(encoding="utf-8")
    assert "require_guards(bool(rospy.get_param" in source


# ------------------------------------------------------------------ contract

def test_the_node_keeps_the_follower_contract():
    """Same node name, topics, service and stop-on-shutdown as the follower,
    so go.sh, stop.sh, the black box and the gate chain are unchanged."""
    source = (SCRIPTS / "priest_follower.py").read_text(encoding="utf-8")
    assert 'rospy.init_node("waypoint_follower")' in source
    assert '"/waypoint_follower/status"' in source
    assert '"/waypoint_follower/start"' in source
    assert '"/cmd_vel_raw"' in source
    assert "rospy.on_shutdown(self.send_stop)" in source
    assert 'String(data="HOLD:" + reason)' in source


def test_the_bringup_defaults_to_the_validated_follower():
    """PRIEST is an opt-in trial. An absent PLANNER variable must launch the
    route follower, and a typo must refuse rather than pick one."""
    bringup = (ROOT.parents[1] / "tools"
               / "start_wheelchair_localization.sh").read_text(
        encoding="utf-8")
    assert 'PLANNER="${PLANNER:-route}"' in bringup
    assert 'FOLLOWER_SCRIPT="waypoint_follower.py"' in bringup
    assert 'FOLLOWER_SCRIPT="priest_follower.py"' in bringup
    assert '[ "$PLANNER" != "route" ] && [ "$PLANNER" != "priest" ]' in bringup


def test_the_speed_and_control_limits_match_the_validated_follower():
    """The gate's ceilings were sized against the follower's limits; a new
    planner that asked for more would be silently clipped one stage down,
    and one that turned faster would be outside the measured envelope."""
    wf = load("waypoint_follower") if False else None  # heavy; read source
    follower = (SCRIPTS / "waypoint_follower.py").read_text(encoding="utf-8")
    for constant in ("MAX_SPEED = 0.6", "MAX_YAW_RATE = 0.5",
                     "MAX_ACCEL = 0.18", "MAX_DECEL = 0.6",
                     "TURN_FLOOR_SPEED = 0.30"):
        assert constant in follower
        assert getattr(pf, constant.split(" =")[0]) == float(
            constant.split("= ")[1])
