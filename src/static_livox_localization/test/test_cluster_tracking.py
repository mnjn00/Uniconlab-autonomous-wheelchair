"""Whether an object is going to move, decided from how it has moved.

The whole avoidance policy rests on this one bit: parked things are driven
around, moving things are waited for. Getting it wrong in one direction
leaves the chair sitting in front of a bollard forever, and in the other it
steers into the space a walking person is about to occupy. Only the second
is dangerous, so every case that cannot be decided has to come out MOVING.
"""

import importlib.util
import math
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


ct = load("cluster_tracking")


def feed(tracker, path, start=100.0, step=0.2, label="obstacle"):
    """Drive one object along `path` and return its final track."""
    track = None
    for i, (x, y) in enumerate(path):
        track = tracker.update([(x, y, label)], start + i * step)[0]
    return track, start + (len(path) - 1) * step


def still(count, x=5.0, y=0.0):
    return [(x, y)] * count


def test_a_new_track_is_unknown_not_parked():
    """The dangerous default. Something seen for the first time has no
    history to be still in, and driving around it on that basis is exactly
    the manoeuvre this must never make."""
    tracker = ct.Tracker()
    track, now = feed(tracker, still(2))
    assert track.motion(now) == ct.UNKNOWN


def test_standing_still_long_enough_becomes_static():
    tracker = ct.Tracker()
    track, now = feed(tracker, still(20))
    assert track.age_s(now) >= ct.CONFIRM_S
    assert track.motion(now) == ct.STATIC


def test_a_walker_is_moving():
    """1.2 m/s across the window - a slow walk, and the case that must not
    be driven around."""
    tracker = ct.Tracker()
    path = [(5.0, -3.0 + 1.2 * 0.2 * i) for i in range(20)]
    track, now = feed(tracker, path, label="person")
    assert track.motion(now) == ct.MOVING


def test_centroid_jitter_does_not_read_as_walking():
    """A partially observed wall's centroid slides as more of it comes into
    view. Judging motion frame to frame would call that a moving object and
    stop the chair for a wall it is driving past."""
    tracker = ct.Tracker()
    path = [(5.0 + 0.03 * math.sin(i), 0.02 * math.cos(i * 1.7))
            for i in range(25)]
    track, now = feed(tracker, path)
    assert track.motion(now) == ct.STATIC


def test_something_that_stops_stops_being_called_moving():
    """A person who crosses and then stands still on the far side has to
    become passable, or the chair waits out someone who is not going
    anywhere."""
    tracker = ct.Tracker()
    walking = [(5.0, -2.0 + 0.3 * i) for i in range(10)]
    track, now = feed(tracker, walking, label="person")
    assert track.motion(now) == ct.MOVING

    parked_at = walking[-1]
    for i in range(1, int(ct.HISTORY_S / 0.2) + 6):
        track = tracker.update([parked_at + ("person",)], now + i * 0.2)[0]
    assert track.motion(now + 0.2 * (int(ct.HISTORY_S / 0.2) + 5)) == ct.STATIC


def test_a_track_that_disappears_comes_back_as_a_new_one():
    """No identity through occlusion, on purpose. Someone who steps behind a
    van and reappears has had time to change what they are doing, and the
    confirmation window has to be paid again."""
    tracker = ct.Tracker()
    track, now = feed(tracker, still(20))
    assert track.motion(now) == ct.STATIC

    gone_for = ct.DROP_AFTER_S + 0.4
    fresh = tracker.update([(5.0, 0.0, "obstacle")], now + gone_for)[0]
    assert fresh.id != track.id
    assert fresh.motion(now + gone_for) == ct.UNKNOWN


def test_two_objects_keep_their_own_identities():
    tracker = ct.Tracker()
    for i in range(12):
        tracks = tracker.update(
            [(5.0, 1.0, "obstacle"), (5.0, -1.0, "person")], 100.0 + i * 0.2)
    assert tracks[0].id != tracks[1].id
    assert tracks[0].label == "obstacle"
    assert tracks[1].label == "person"


def test_association_prefers_the_nearer_pairing_not_the_listed_order():
    """Detections arrive sorted by cluster size, which says nothing about
    which object is which. Assigning in that order lets a big cluster claim
    a track belonging to a small one standing much closer to it."""
    tracker = ct.Tracker()
    tracker.update([(5.0, 0.0, "a"), (5.6, 0.0, "b")], 100.0)
    first_id = tracker.tracks[0].id
    second_id = tracker.tracks[1].id

    # Same two objects, reported in the other order.
    tracks = tracker.update([(5.6, 0.0, "b"), (5.05, 0.0, "a")], 100.2)
    assert tracks[0].id == second_id
    assert tracks[1].id == first_id


def test_an_object_beyond_the_gate_is_not_the_same_object():
    tracker = ct.Tracker()
    tracker.update([(5.0, 0.0, "obstacle")], 100.0)
    far = tracker.update(
        [(5.0 + ct.ASSOCIATION_GATE_M + 0.5, 0.0, "obstacle")], 100.2)[0]
    assert far.motion(100.2) == ct.UNKNOWN
    assert len(tracker.tracks) == 2


@pytest.mark.parametrize("speed", [0.0, 0.05, 0.19])
def test_slow_drift_stays_parked(speed):
    tracker = ct.Tracker()
    path = [(5.0 + speed * 0.2 * i, 0.0) for i in range(20)]
    track, now = feed(tracker, path)
    assert track.motion(now) == ct.STATIC


@pytest.mark.parametrize("speed", [0.25, 0.6, 1.5])
def test_anything_faster_than_the_threshold_is_moving(speed):
    tracker = ct.Tracker()
    path = [(5.0 + speed * 0.2 * i, 0.0) for i in range(20)]
    track, now = feed(tracker, path)
    assert track.motion(now) == ct.MOVING


# ------------------------------------------------------------- the frame chain
# Everything above assumes odom-frame input. The producer has to actually
# produce that, out of lidar-frame cluster centres and a body pose, and if it
# gets the composition wrong then a parked car "moves" at driving speed and
# the chair waits for every wall it passes.

def test_a_parked_object_does_not_move_while_the_chair_drives_past_it():
    import numpy as np
    bf = load("body_frame")
    lidar_in_body, rotation = bf.lidar_extrinsics("vn100")

    fixed_in_odom = np.array([10.0, 1.0, 0.0])
    seen = []
    for step in range(12):     # 2.2 s, past CONFIRM_S, over 6.6 m of driving
        # Chair driving up +x and yawing slightly, as it does on a bend.
        yaw = 0.05 * step
        T = np.eye(4)
        T[:3, :3] = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                              [math.sin(yaw), math.cos(yaw), 0.0],
                              [0.0, 0.0, 1.0]])
        T[:3, 3] = (0.6 * step, 0.0, 0.0)

        # What the producer would see: the object in its own lidar frame.
        in_body = np.linalg.inv(T) @ np.append(fixed_in_odom, 1.0)
        in_lidar = bf.body_to_lidar(in_body[:3].reshape(1, 3),
                                    lidar_in_body, rotation)

        # ... and the chain track() puts it back through.
        back_body = bf.lidar_to_body(in_lidar, lidar_in_body, rotation)
        back_odom = back_body @ T[:3, :3].T + T[:3, 3]
        seen.append(back_odom[0][:2])

    drift = max(float(np.linalg.norm(p - seen[0])) for p in seen)
    assert drift < 1e-9, "a parked object drifted %.4f m in odom" % drift

    tracker = ct.Tracker()
    track = None
    for i, point in enumerate(seen):
        track = tracker.update(
            [(float(point[0]), float(point[1]), "vehicle")], 100.0 + i * 0.2)[0]
    assert track.motion(100.0 + 0.2 * (len(seen) - 1)) == ct.STATIC
