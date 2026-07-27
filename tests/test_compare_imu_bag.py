"""Pure regression coverage for the dual-IMU bag comparison math.

No bag and no rosbags install are needed: the reader is lazy-imported, so
only the numpy core is exercised here.
"""
import importlib.util
import math
import unittest
from pathlib import Path

import numpy as np

MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "compare_imu_bag.py"
SPEC = importlib.util.spec_from_file_location("compare_imu_bag", str(MODULE_PATH))
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


class YawRotationTest(unittest.TestCase):
    def test_positive_yaw_carries_x_onto_y(self):
        rot = module.yaw_rotation(90.0)
        out = rot @ np.array([1.0, 0.0, 0.0])
        np.testing.assert_allclose(out, [0.0, 1.0, 0.0], atol=1e-12)

    def test_zero_yaw_is_identity(self):
        np.testing.assert_allclose(module.yaw_rotation(0.0), np.eye(3), atol=1e-12)


class StationaryBlocksTest(unittest.TestCase):
    def test_short_blocks_are_dropped_and_indices_are_half_open(self):
        t = np.arange(0.0, 12.0)
        speed = np.zeros(12)
        speed[4:6] = 1.0  # stationary 0-3 (4 s), 6-7 (2 s), 9-11 (3 s)
        speed[8] = 1.0
        self.assertEqual(module.stationary_blocks(t, speed, 0.02, 2.5), [(0, 4)])
        self.assertEqual(
            module.stationary_blocks(t, speed, 0.02, 1.0),
            [(0, 4), (6, 8), (9, 12)],
        )

    def test_empty_input_gives_no_blocks(self):
        self.assertEqual(module.stationary_blocks(np.array([]), np.array([]), 0.02, 20.0), [])

    def test_all_stationary_is_one_block(self):
        t = np.linspace(0.0, 30.0, 31)
        blocks = module.stationary_blocks(t, np.zeros(31), 0.02, 20.0)
        self.assertEqual(blocks, [(0, 31)])


class GyroStatsTest(unittest.TestCase):
    def test_constant_rate_integrates_to_angle_times_duration(self):
        t = np.linspace(0.0, 10.0, 1001)
        gyro = np.tile(np.array([0.0, 0.0, 0.1]), (len(t), 1))
        stats = module.gyro_stats(t, gyro, 0, len(t))
        np.testing.assert_allclose(stats["bias"], [0.0, 0.0, 0.1], atol=1e-12)
        np.testing.assert_allclose(stats["noise"], 0.0, atol=1e-12)
        np.testing.assert_allclose(stats["integrated"], [0.0, 0.0, 1.0], atol=1e-9)


class AccelScaleTest(unittest.TestCase):
    def test_labels_match_the_two_known_sensors(self):
        self.assertEqual(module.accel_scale_label(0.9994), "g")
        self.assertEqual(module.accel_scale_label(9.5435), "m/s^2")
        self.assertEqual(module.accel_scale_label(5.0), "unknown")


class GravityAndTiltTest(unittest.TestCase):
    def test_level_gives_zero_tilt(self):
        norm, pitch, roll = module.gravity_and_tilt(np.array([0.0, 0.0, 9.80665]))
        self.assertAlmostEqual(norm, 9.80665, places=5)
        self.assertAlmostEqual(pitch, 0.0, places=9)
        self.assertAlmostEqual(roll, 0.0, places=9)

    def test_pitch_tilt_recovers_angle(self):
        g = 9.80665
        accel = np.array([-g * math.sin(math.radians(5.0)), 0.0, g * math.cos(math.radians(5.0))])
        _norm, pitch, roll = module.gravity_and_tilt(accel)
        self.assertAlmostEqual(pitch, 5.0, places=9)
        self.assertAlmostEqual(roll, 0.0, places=9)


class AlignmentAndAgreementTest(unittest.TestCase):
    def test_align_resamples_onto_reference_stamps(self):
        t_ref = np.array([0.5, 1.5])
        t_src = np.array([0.0, 1.0, 2.0])
        values = np.column_stack([t_src * 2.0, np.zeros(3), np.ones(3)])
        out = module.align_onto(t_ref, t_src, values)
        np.testing.assert_allclose(out[:, 0], [1.0, 3.0], atol=1e-12)
        np.testing.assert_allclose(out[:, 2], 1.0, atol=1e-12)

    def test_rms_and_correlation_on_known_series(self):
        a = np.array([[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
        b = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
        np.testing.assert_allclose(module.rms_per_axis(a - b), [np.sqrt(14.0 / 3.0), 0.0])
        self.assertAlmostEqual(module.correlation(a[:, 0], a[:, 0] * 2.0), 1.0, places=12)
        self.assertTrue(math.isnan(module.correlation(b[:, 0], a[:, 0])))


if __name__ == "__main__":
    unittest.main()
