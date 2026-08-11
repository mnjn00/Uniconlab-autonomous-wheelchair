"""Intensity-based retroreflector blooming filter tests.

Drives the real obstacle_clusters step() with synthetic (x,y,z,intensity)
clouds to verify that saturated-intensity points (traffic signs, mirror
surfaces) are removed before clustering and never become phantom obstacles.
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


cp = load("cloud_points")


class FakeField:
    def __init__(self, name, offset, datatype=7, count=1):
        self.name = name
        self.offset = offset
        self.datatype = datatype
        self.count = count


class FakeCloud:
    def __init__(self, points):
        dtype = np.dtype([
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
            ("intensity", "<f4"),
        ])
        records = np.empty(len(points), dtype=dtype)
        records["x"] = points[:, 0]
        records["y"] = points[:, 1]
        records["z"] = points[:, 2]
        if points.shape[1] >= 4:
            records["intensity"] = points[:, 3]
        else:
            records["intensity"] = 0.0
        self.fields = [
            FakeField("x", 0), FakeField("y", 4),
            FakeField("z", 8), FakeField("intensity", 12),
        ]
        self.is_bigendian = False
        self.point_step = 16
        self.row_step = 16 * len(points)
        self.width = len(points)
        self.height = 1
        self.data = records.tobytes()


class FakeCloudNoIntensity:
    def __init__(self, points):
        dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
        records = np.empty(len(points), dtype=dtype)
        records["x"] = points[:, 0]
        records["y"] = points[:, 1]
        records["z"] = points[:, 2]
        self.fields = [
            FakeField("x", 0), FakeField("y", 4), FakeField("z", 8),
        ]
        self.is_bigendian = False
        self.point_step = 12
        self.row_step = 12 * len(points)
        self.width = len(points)
        self.height = 1
        self.data = records.tobytes()


class Stamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    @property
    def last(self):
        return self.messages[-1]


class Loose:
    CUBE = 1
    ADD = 0
    DELETEALL = 3

    def __init__(self, *a, **k):
        pass

    def __getattr__(self, name):
        child = Loose()
        object.__setattr__(self, name, child)
        return child


class Markers:
    def __init__(self):
        self.markers = []


def load_producer(now_s):
    rospy = types.ModuleType("rospy")
    rospy.init_node = lambda *a, **k: None
    rospy.get_param = lambda name, default=None: default
    rospy.Publisher = lambda *a, **k: Capture()
    rospy.Subscriber = lambda *a, **k: None
    rospy.Duration = lambda s: s
    rospy.Rate = lambda hz: None
    rospy.is_shutdown = lambda: True
    rospy.loginfo = rospy.logwarn = lambda *a, **k: None
    rospy.Time = types.SimpleNamespace(now=lambda: Stamp(now_s[0]))

    modules = {"rospy": rospy}
    for pkg, names in {
        "geometry_msgs.msg": ["Point", "PoseWithCovarianceStamped"],
        "nav_msgs.msg": ["Odometry"],
        "sensor_msgs.msg": ["PointCloud2"],
        "std_msgs.msg": ["String"],
        "visualization_msgs.msg": ["Marker"],
    }.items():
        m = types.ModuleType(pkg)
        for n in names:
            setattr(m, n, Loose)
        modules[pkg] = m
        root = pkg.split(".")[0]
        modules.setdefault(root, types.ModuleType(root))
        setattr(modules[root], pkg.split(".")[1], m)
    modules["visualization_msgs.msg"].MarkerArray = Markers
    modules["std_msgs.msg"].String = lambda data: types.SimpleNamespace(
        data=data)

    pc2 = types.ModuleType("sensor_msgs.point_cloud2")
    pc2.read_points = lambda *a, **k: []
    modules["sensor_msgs.point_cloud2"] = pc2
    modules["sensor_msgs"].point_cloud2 = pc2

    tr = types.ModuleType("tf.transformations")
    tr.quaternion_matrix = lambda v: np.eye(4)
    tf = types.ModuleType("tf")
    tf.transformations = tr
    modules["tf"] = tf
    modules["tf.transformations"] = tr

    saved = {n: sys.modules.get(n) for n in modules}
    sys.modules.update(modules)
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            "obstacle_clusters_reflection_test",
            SCRIPTS / "obstacle_clusters.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))
        for n, p in saved.items():
            if p is None:
                sys.modules.pop(n, None)
            else:
                sys.modules[n] = p


ct = load("cluster_tracking")


def producer_at(now_s):
    module = load_producer(now_s)
    node = module.ObstacleClusters.__new__(module.ObstacleClusters)
    lidar_in_body, rotation = module.lidar_extrinsics("vn100")
    node.accumulator = module.Accumulator(lidar_in_body, rotation)
    node.lidar_in_body = lidar_in_body
    node.lidar_to_body_rotation = rotation
    node.tracker = ct.Tracker()
    node.band = None
    node.band_grace_m = module.OBJECT_BAND_GRACE_M
    node.map_poses = module.MapPoseBuffer()
    node.marker_pub = Capture()
    node.dynamic_pub = Capture()
    node.summary_pub = Capture()
    node._last_bloom_removed = 0
    return module, node


def box_xyzi(cx, cy, half=0.25, height=1.2, intensity=30.0):
    xs = np.arange(cx - half, cx + half, 0.05)
    ys = np.arange(cy - half, cy + half, 0.05)
    zs = np.arange(0.2, height, 0.1) - 0.30
    return np.array(
        [(x, y, z, intensity) for x in xs for y in ys for z in zs],
        dtype=np.float32)


def run(node, module, cloud, now_s, stamp):
    T = np.eye(4)
    node.accumulator.scans = [(stamp, cloud)]
    node.accumulator.odoms = [(stamp, T)]
    now_s[0] = stamp
    node.step()
    return json.loads(node.summary_pub.last.data)


# --- cloud_points.points_xyzi ---------------------------------------------

def test_points_xyzi_reads_four_columns():
    pts = np.array([
        [1.0, 2.0, 3.0, 50.0],
        [4.0, 5.0, 6.0, 255.0],
    ], dtype=np.float32)
    out = cp.points_xyzi(FakeCloud(pts))
    assert out.shape == (2, 4)
    assert out[0, 3] == pytest.approx(50.0)
    assert out[1, 3] == pytest.approx(255.0)


def test_points_xyzi_returns_zeros_when_intensity_absent():
    pts = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    out = cp.points_xyzi(FakeCloudNoIntensity(pts))
    assert out.shape == (1, 4)
    assert out[0, 3] == pytest.approx(0.0)


def test_points_xyzi_drops_nan_coordinates():
    pts = np.array([
        [1.0, 2.0, 3.0, 50.0],
        [float("nan"), 5.0, 6.0, 255.0],
    ], dtype=np.float32)
    out = cp.points_xyzi(FakeCloud(pts))
    assert out.shape == (1, 4)


# --- obstacle_clusters intensity filter -----------------------------------

def test_retroreflector_points_are_filtered_out():
    now_s = [100.0]
    module, node = producer_at(now_s)
    bright = box_xyzi(4.0, 0.0, intensity=255.0)
    summary = run(node, module, bright, now_s, 100.0)
    assert summary["bloom_filtered"] > 0
    assert len(summary["objects"]) == 0

    now_s = [100.0]
    module, node = producer_at(now_s)
    normal = box_xyzi(4.0, 0.0, intensity=30.0)
    summary = run(node, module, normal, now_s, 100.0)
    assert summary["bloom_filtered"] == 0
    assert len(summary["objects"]) > 0


def test_bloom_filtered_count_is_reported():
    now_s = [100.0]
    module, node = producer_at(now_s)
    normal = box_xyzi(4.0, 0.0, intensity=30.0)
    bright = box_xyzi(4.0, 0.0, intensity=255.0)
    combined = np.concatenate([normal, bright], axis=0)
    summary = run(node, module, combined, now_s, 100.0)
    assert "bloom_filtered" in summary
    assert summary["bloom_filtered"] > 0


def test_intensity_threshold_is_at_saturation_boundary():
    assert cp is not None
    module = load_producer([100.0])
    assert module.RETROREFLECTOR_INTENSITY == 200.0

    now_s = [100.0]
    module, node = producer_at(now_s)
    dim = box_xyzi(4.0, 0.0, intensity=199.0)
    summary = run(node, module, dim, now_s, 100.0)
    assert summary["bloom_filtered"] == 0
    assert len(summary["objects"]) > 0

    now_s = [100.0]
    module, node = producer_at(now_s)
    bright = box_xyzi(4.0, 0.0, intensity=200.0)
    summary = run(node, module, bright, now_s, 100.0)
    assert summary["bloom_filtered"] > 0
    assert len(summary["objects"]) == 0


def test_mixed_scene_stops_only_real_objects():
    now_s = [100.0]
    module, node = producer_at(now_s)
    real = box_xyzi(3.0, 0.0, intensity=40.0)
    sign = box_xyzi(5.0, 0.0, intensity=250.0)
    combined = np.concatenate([real, sign], axis=0)
    summary = run(node, module, combined, now_s, 100.0)
    assert len(summary["objects"]) == 1
    assert summary["objects"][0]["x"] < 4.0
