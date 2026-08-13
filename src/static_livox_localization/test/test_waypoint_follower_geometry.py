import importlib.util
import sys
import types
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).parents[1] / "scripts"


def load_follower_module():
    dummy = type("Dummy", (), {})
    rospy = types.ModuleType("rospy")
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.logwarn = lambda *args, **kwargs: None
    rospy.logwarn_throttle = lambda *args, **kwargs: None
    rospy.logerr_throttle = lambda *args, **kwargs: None

    modules = {"rospy": rospy}
    for package, names in {
        "diagnostic_msgs.msg": ["DiagnosticArray"],
        "geometry_msgs.msg": ["PoseWithCovarianceStamped", "Twist"],
        "nav_msgs.msg": ["Odometry"],
        "sensor_msgs.msg": ["PointCloud2"],
        "std_msgs.msg": ["Int16MultiArray", "String"],
        "std_srvs.srv": ["SetBool", "SetBoolResponse"],
    }.items():
        module = types.ModuleType(package)
        for name in names:
            setattr(module, name, dummy)
        modules[package] = module
        root = package.split(".")[0]
        modules.setdefault(root, types.ModuleType(root))
        setattr(modules[root], package.split(".")[1], module)

    point_cloud = types.ModuleType("sensor_msgs.point_cloud2")
    point_cloud.read_points = lambda *args, **kwargs: []
    modules["sensor_msgs.point_cloud2"] = point_cloud
    modules["sensor_msgs"].point_cloud2 = point_cloud

    transformations = types.ModuleType("tf.transformations")
    transformations.quaternion_matrix = lambda value: np.eye(4)
    transformations.euler_from_quaternion = lambda value: (0.0, 0.0, 0.0)
    tf = types.ModuleType("tf")
    tf.transformations = transformations
    modules["tf"] = tf
    modules["tf.transformations"] = transformations

    saved = {name: sys.modules.get(name) for name in modules}
    sys.modules.update(modules)
    sys.path.insert(0, str(SCRIPT_DIR))
    try:
        spec = importlib.util.spec_from_file_location(
            "waypoint_follower_geometry_test", SCRIPT_DIR / "waypoint_follower.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(SCRIPT_DIR))
        for name, prior in saved.items():
            if prior is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = prior
    return module


class DistanceBand:
    def __init__(self, safe_distance):
        self.safe_distance = safe_distance

    def clamp(self, target):
        return target

    def recentre(self, target):
        """This double models chord distance only, so the safe-side lean is
        a no-op here - the lean's own behaviour is covered in
        test_safety_band."""
        return target

    def chord_is_contained(self, start, target, grace=0.0):
        return np.linalg.norm(target - start) <= self.safe_distance + 1e-9


def follower_for(module, safe_distance):
    follower = module.WaypointFollower.__new__(module.WaypointFollower)
    follower.waypoints = np.array([[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]])
    follower.pose_xy = np.array([0.0, 0.0])
    follower.nearest_index = 0
    follower.lateral_offset = 0.0
    follower.band = DistanceBand(safe_distance)
    follower.drivable_mask = type(
        "Mask", (), {"contains": staticmethod(lambda _point: True)}
    )()
    # The guarded configuration: these are the band's own geometry tests, and
    # with the policies off there is no band to test.
    follower.policies = True
    return follower


def test_lookahead_interpolates_inside_a_sparse_route_segment():
    module = load_follower_module()
    follower = follower_for(module, 99.0)
    np.testing.assert_allclose(follower.lookahead_point(1.25), [1.25, 0.0])


def test_lookahead_starts_at_the_chairs_projection_not_the_prior_waypoint():
    module = load_follower_module()
    follower = follower_for(module, 99.0)
    follower.pose_xy = np.array([2.5, 0.2])
    np.testing.assert_allclose(follower.lookahead_point(1.25), [3.75, 0.0])


def test_lookahead_crosses_sparse_segment_boundaries_by_distance():
    module = load_follower_module()
    follower = follower_for(module, 99.0)
    follower.pose_xy = np.array([4.5, 0.0])
    follower.nearest_index = 1
    np.testing.assert_allclose(follower.lookahead_point(1.25), [5.75, 0.0])


def test_safe_target_backs_off_and_caps_speed():
    module = load_follower_module()
    follower = follower_for(module, 1.05)
    target, cap, safe = follower.safe_target(1.8)
    assert safe
    np.testing.assert_allclose(target, [1.0, 0.0])
    assert cap == module.CREEP_SPEED


def test_no_safe_chord_returns_a_zero_speed_hold():
    module = load_follower_module()
    follower = follower_for(module, 0.2)
    _target, cap, safe = follower.safe_target(1.8)
    assert not safe
    assert cap == 0.0
