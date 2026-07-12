"""ROS-free regression tests for the perception node IMU cache."""

from collections import deque
import importlib.util
from pathlib import Path
import sys
import threading
import time
import types
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "perception_node.py"
SCRIPT_DIR = str(SCRIPT.parent)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
SPEC = importlib.util.spec_from_file_location("perception_node_imu_cache", SCRIPT)
perception_node = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = perception_node
SPEC.loader.exec_module(perception_node)


class Stamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


def imu_message(stamp):
    vector = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=Stamp(stamp), frame_id="imu_link"),
        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        linear_acceleration=vector,
        angular_velocity=vector,
    )


class SlowDeque(deque):
    """Expose deque iteration long enough for a concurrent append attempt."""

    def __init__(self, values, started):
        super().__init__(values, maxlen=values.maxlen)
        self.started = started

    def __iter__(self):
        iterator = super().__iter__()
        for value in iterator:
            self.started.set()
            time.sleep(0.002)
            yield value


class ImuCacheTests(unittest.TestCase):
    def test_concurrent_append_cannot_mutate_alignment_iteration(self):
        cache = perception_node.ImuCache(max_skew_s=100.0, capacity=128)
        for stamp in range(64):
            cache.add(imu_message(float(stamp + 1)))

        iteration_started = threading.Event()
        with cache._lock:
            cache._samples = SlowDeque(cache._samples, iteration_started)

        errors = []
        result = []

        def align():
            try:
                result.append(cache.aligned(32.25))
            except Exception as error:  # Regression assertion records worker failures.
                errors.append(error)

        reader = threading.Thread(target=align)
        reader.start()
        self.assertTrue(iteration_started.wait(timeout=1.0))
        writer = threading.Thread(target=lambda: cache.add(imu_message(65.0)))
        writer.start()
        reader.join(timeout=2.0)
        writer.join(timeout=2.0)

        self.assertFalse(reader.is_alive())
        self.assertFalse(writer.is_alive())
        self.assertEqual([], errors)
        self.assertEqual(32.0, result[0].stamp_s)

    def test_capacity_retains_only_newest_samples_in_append_order(self):
        cache = perception_node.ImuCache(max_skew_s=100.0, capacity=3)
        for stamp in (10.0, 12.0, 11.0, 13.0):
            cache.add(imu_message(stamp))

        with cache._lock:
            retained = tuple(item.stamp_s for item in cache._samples)
        self.assertEqual((12.0, 11.0, 13.0), retained)
        self.assertEqual(11.0, cache.aligned(11.0).stamp_s)

    def test_nearest_selection_is_deterministic_for_ties_and_regression(self):
        cache = perception_node.ImuCache(max_skew_s=2.0)
        first = cache.add(imu_message(11.0))
        cache.add(imu_message(9.0))

        self.assertIs(first, cache.aligned(10.0))

    def test_empty_stale_future_and_boundary_timestamps(self):
        cache = perception_node.ImuCache(max_skew_s=0.5)
        self.assertIsNone(cache.aligned(10.0))
        sample = cache.add(imu_message(10.0))

        self.assertIs(sample, cache.aligned(9.5))
        self.assertIs(sample, cache.aligned(10.5))
        self.assertIsNone(cache.aligned(9.499))
        self.assertIsNone(cache.aligned(10.501))


if __name__ == "__main__":
    unittest.main()
