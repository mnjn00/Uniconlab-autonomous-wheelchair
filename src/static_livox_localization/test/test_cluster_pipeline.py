"""The producer and the consumer, joined at the topic they share.

Both sides are tested apart: test_cluster_tracking on the verdict, and
test_cluster_guard on the reading. Neither notices if the producer stops
writing the field the consumer reads, or writes it under another name, or
publishes centres where the consumer expects near faces - and any of those
turns the guard into one that reports clear road forever, with every unit
test still green.

So this drives the real node's step() over a synthetic scene and puts the
JSON it publishes through the real parser.
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


cg = load("cluster_guard")
ct = load("cluster_tracking")


class Capture(object):
    """A publisher that keeps what it was given."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    @property
    def last(self):
        return self.messages[-1]


class Loose(object):
    """Stands in for a message type by accepting whatever is set on it."""

    CUBE = 1
    ADD = 0
    DELETEALL = 3

    def __init__(self, *args, **kwargs):
        if args:
            self.value = args[0]

    def __getattr__(self, name):
        child = Loose()
        object.__setattr__(self, name, child)
        return child


class Markers(object):
    def __init__(self):
        self.markers = []


class Stamp(object):
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


def load_producer(now_s):
    """obstacle_clusters with ROS replaced by the little of it that is used."""
    rospy = types.ModuleType("rospy")
    rospy.init_node = lambda *a, **k: None
    rospy.get_param = lambda name, default=None: default
    rospy.Publisher = lambda *a, **k: Capture()
    rospy.Subscriber = lambda *a, **k: None
    rospy.Duration = lambda seconds: seconds
    rospy.Rate = lambda hz: None
    rospy.is_shutdown = lambda: True
    rospy.loginfo = rospy.logwarn = lambda *a, **k: None
    rospy.Time = types.SimpleNamespace(now=lambda: Stamp(now_s[0]))

    modules = {"rospy": rospy}
    for package, names in {
        "geometry_msgs.msg": ["Point", "PoseWithCovarianceStamped"],
        "nav_msgs.msg": ["Odometry"],
        "sensor_msgs.msg": ["PointCloud2"],
        "std_msgs.msg": ["String"],
        "visualization_msgs.msg": ["Marker"],
    }.items():
        module = types.ModuleType(package)
        for name in names:
            setattr(module, name, Loose)
        modules[package] = module
        root = package.split(".")[0]
        modules.setdefault(root, types.ModuleType(root))
        setattr(modules[root], package.split(".")[1], module)
    modules["visualization_msgs.msg"].MarkerArray = Markers
    modules["std_msgs.msg"].String = lambda data: types.SimpleNamespace(
        data=data)

    point_cloud = types.ModuleType("sensor_msgs.point_cloud2")
    point_cloud.read_points = lambda *a, **k: []
    modules["sensor_msgs.point_cloud2"] = point_cloud
    modules["sensor_msgs"].point_cloud2 = point_cloud

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
            "obstacle_clusters_pipeline_test", SCRIPTS / "obstacle_clusters.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(SCRIPTS))
        for name, prior in saved.items():
            if prior is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = prior
    return module


def box_of_points(centre_x, centre_y, half=0.25, height=1.2):
    """A dense upright box, enough points to survive the cell thresholds."""
    xs = np.arange(centre_x - half, centre_x + half, 0.05)
    ys = np.arange(centre_y - half, centre_y + half, 0.05)
    zs = np.arange(0.2, height, 0.1) - 0.30      # relative to the sensor
    grid = np.array([(x, y, z) for x in xs for y in ys for z in zs])
    return grid.astype(np.float32)


def wall_of_points(start_x=3.0, end_x=5.0, centre_y=0.0, height=1.2):
    """A thin connected wall segment that the old heuristic calls vehicle."""
    xs = np.arange(start_x, end_x, 0.05)
    ys = np.arange(centre_y - 0.08, centre_y + 0.08, 0.05)
    zs = np.arange(0.2, height, 0.1) - 0.30
    return np.array([(x, y, z) for x in xs for y in ys for z in zs],
                    dtype=np.float32)


def points_at_ground_heights(ground_heights, centre_x=4.0, centre_y=0.0):
    """A dense object whose z values are stated relative to the ground."""
    xs = np.arange(centre_x - 0.25, centre_x + 0.25, 0.05)
    ys = np.arange(centre_y - 0.25, centre_y + 0.25, 0.05)
    return np.array([
        (x, y, height - 0.725)
        for x in xs for y in ys for height in ground_heights
    ], dtype=np.float32)


class ConstantBand(object):
    def __init__(self, contained):
        self.contained = contained

    def contains_many(self, points, grace=0.0):
        return np.full(len(points), self.contained, dtype=bool)


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
    node.summary_pub = Capture()
    return module, node


def run(node, module, cloud, now_s, pose_x, stamp):
    """One producer cycle with the chair at pose_x and the given cloud."""
    T = np.eye(4)
    T[:3, 3] = (pose_x, 0.0, 0.0)
    node.accumulator.scans = [(stamp, cloud)]
    node.accumulator.odoms = [(stamp, T)]
    now_s[0] = stamp
    node.step()
    return cg.parse_summary(node.summary_pub.last.data)


def test_the_producer_writes_what_the_consumer_reads():
    now_s = [100.0]
    module, node = producer_at(now_s)
    summary = run(node, module, box_of_points(4.0, 0.0), now_s, 0.0, 100.0)

    assert summary.usable
    assert summary.objects, "no object came out of a box of points"
    first = summary.objects[0]
    for field in ("class", "x", "y", "size", "motion", "id", "speed_mps"):
        assert field in first, "producer stopped writing %s" % field

    threat = cg.nearest_threat(summary, 0.45)
    assert threat is not None
    # Near face of a 0.5 m box centred at 4 m, seen through the extrinsic.
    assert threat.distance_m == pytest.approx(3.75, abs=0.25)


def test_overhead_only_returns_cannot_become_a_control_object():
    """Raising the cluster ceiling above the gate's 1.50 m limit must fail."""
    now_s = [100.0]
    module, node = producer_at(now_s)
    overhead = points_at_ground_heights(np.arange(1.60, 1.91, 0.10))

    summary = run(node, module, overhead, now_s, 0.0, 100.0)

    assert summary.usable
    assert summary.objects == []


def test_mixed_height_object_keeps_its_collision_relevant_lower_returns():
    """Filtering overhead must not discard an object that extends below it."""
    now_s = [100.0]
    module, node = producer_at(now_s)
    mixed = points_at_ground_heights(
        np.concatenate((np.arange(1.10, 1.41, 0.10),
                        np.arange(1.60, 1.91, 0.10))))

    summary = run(node, module, mixed, now_s, 0.0, 100.0)

    assert summary.objects, "the lower collision-relevant returns disappeared"
    assert max(item["size"][2] for item in summary.objects) <= 0.31


def test_a_first_sighting_is_not_reported_as_parked():
    now_s = [100.0]
    module, node = producer_at(now_s)
    summary = run(node, module, box_of_points(4.0, 0.0), now_s, 0.0, 100.0)

    assert cg.nearest_threat(summary, 0.45).motion == ct.UNKNOWN


def test_a_bollard_the_chair_drives_up_to_is_reported_parked():
    """The whole point of the frame chain: the object closes at driving
    speed in the lidar frame and does not move at all in odom."""
    now_s = [100.0]
    module, node = producer_at(now_s)

    summary = None
    for step in range(14):                       # 2.6 s, past CONFIRM_S
        pose_x = 0.12 * step                     # 0.6 m/s at 5 Hz
        summary = run(node, module, box_of_points(4.0 - pose_x, 0.0),
                      now_s, pose_x, 100.0 + step * 0.2)

    threat = cg.nearest_threat(summary, 0.45)
    assert threat is not None
    assert threat.motion == ct.STATIC, \
        "a parked bollard read as %s - the odom transform is wrong" % \
        threat.motion
    assert threat.parked


def test_someone_walking_across_is_not_reported_parked():
    now_s = [100.0]
    module, node = producer_at(now_s)

    summary = None
    for step in range(14):
        pose_x = 0.12 * step
        across = -1.5 + 0.24 * step              # 1.2 m/s across the path
        summary = run(node, module, box_of_points(4.0 - pose_x, across),
                      now_s, pose_x, 100.0 + step * 0.2)

    walker = [o for o in summary.objects if o["motion"] == ct.MOVING]
    assert walker, "a 1.2 m/s crossing read as parked: %s" % summary.objects


def test_no_reference_pose_reports_unknown_rather_than_parked():
    """No odometry means no frame to judge motion in. Reporting stillness
    from a frame that is itself moving is how a wall becomes passable."""
    now_s = [100.0]
    module, node = producer_at(now_s)
    node.accumulator.scans = [(100.0, box_of_points(4.0, 0.0))]
    node.accumulator.odoms = []
    node.step()

    summary = cg.parse_summary(node.summary_pub.last.data)
    assert summary.status == "NO_CLOUD"
    assert not cg.nearest_threat(summary, 0.45).parked


def test_the_summary_says_which_frame_its_numbers_are_in():
    """The consumer's clearance constants are lidar-frame. The markers next
    to them are drawn in "body", 0.14 m away, and nothing but this field
    distinguishes the two."""
    now_s = [100.0]
    module, node = producer_at(now_s)
    run(node, module, box_of_points(4.0, 0.0), now_s, 0.0, 100.0)

    assert json.loads(node.summary_pub.last.data)["frame"] == "lidar"


def test_a_wall_outside_the_band_is_not_published_as_a_vehicle():
    now_s = [100.0]
    module, node = producer_at(now_s)
    node.band = ConstantBand(False)
    node.map_poses.add(100.0, np.eye(4))

    summary = run(node, module, wall_of_points(), now_s, 0.0, 100.0)
    wall = max(summary.objects, key=lambda item: item["points"])

    assert wall["raw_class"] == "vehicle", \
        "the fixture no longer reproduces the wall-as-vehicle defect"
    assert wall["class"] == module.OUTSIDE_BAND
    assert wall["band_relation"] == "outside"
    assert wall["band_inside_fraction"] == 0.0


def test_an_object_inside_the_band_keeps_its_semantic_class():
    now_s = [100.0]
    module, node = producer_at(now_s)
    node.band = ConstantBand(True)
    node.map_poses.add(100.0, np.eye(4))

    summary = run(node, module, wall_of_points(), now_s, 0.0, 100.0)
    item = max(summary.objects, key=lambda value: value["points"])

    assert item["raw_class"] == "vehicle"
    assert item["class"] == "vehicle"
    assert item["band_relation"] == "inside"


def test_missing_map_pose_never_hides_the_original_detection():
    now_s = [100.0]
    module, node = producer_at(now_s)
    node.band = ConstantBand(False)

    summary = run(node, module, wall_of_points(), now_s, 0.0, 100.0)
    item = max(summary.objects, key=lambda value: value["points"])

    assert item["class"] == "vehicle"
    assert item["band_relation"] == "unavailable"
    assert json.loads(node.summary_pub.last.data)["band_status"] == \
        "NO_MAP_POSE"


def test_field_startup_passes_the_same_band_as_the_follower():
    startup = (SCRIPTS.parents[2] / "tools" /
               "start_wheelchair_localization.sh").read_text(encoding="utf-8")
    command = startup.split("obstacle_clusters.py", 1)[1].split(
        "> \"$LOG/live_clusters.log\"", 1)[0]
    assert '_safety_band:="$BAND"' in command
