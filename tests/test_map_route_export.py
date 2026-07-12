"""Synthetic, ROS-free tests for deterministic GLIM map/route export."""

import argparse
import hashlib
import importlib.util
import json
import tempfile
import unittest
from unittest import mock
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("export_glim", ROOT / "scripts" / "export_glim_2d_map.py")
EXPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXPORT)


class MapRouteExportTests(unittest.TestCase):
    def test_grade_uses_horizontal_distance_not_3d_distance(self):
        points = np.array([[0.0, 0.0, 0.0], [0.8, 0.0, 0.6]])
        grade = EXPORT.grade_statistics(points)
        self.assertAlmostEqual(grade["max_grade_percent"], 75.0)
        self.assertEqual(grade["formula"], "100*abs(dz)/sqrt(dx^2+dy^2)")

    def test_time_order_nonfinite_and_duplicate_poses_are_rejected(self):
        cloud = np.array([[0.0, 0.0, 0.0]])
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            EXPORT.validate_inputs(cloud, np.array([[2.0, 0, 0, 0], [1.0, 1, 0, 0]]), True)
        with self.assertRaisesRegex(ValueError, "finite"):
            EXPORT.validate_inputs(np.array([[np.nan, 0, 0]]), np.array([[0, 0, 0], [1, 0, 0]]))
        with self.assertRaisesRegex(ValueError, "duplicate"):
            EXPORT.validate_inputs(cloud, np.array([[0, 0, 0], [0, 0, 0]]))

    def test_tum_trajectory_requires_unit_quaternions(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "traj_lidar.txt"
            np.savetxt(path, [[1.0, 0, 0, 0, 0, 0, 0, 1],
                              [2.0, 1, 0, 0, 0, 0, 0, 1]])
            trajectory, has_time, format_name = EXPORT.load_trajectory(path)
            self.assertTrue(has_time)
            self.assertEqual(format_name, "tum")
            np.testing.assert_allclose(trajectory[:, 1:4], [[0, 0, 0], [1, 0, 0]])
            np.savetxt(path, [[1.0, 0, 0, 0, 0, 0, 0, 2],
                              [2.0, 1, 0, 0, 0, 0, 0, 1]])
            with self.assertRaisesRegex(ValueError, "unit quaternions"):
                EXPORT.load_trajectory(path)

    def test_glim_dump_submaps_are_sorted_and_transformed_to_world(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            transforms = {
                "000002": np.array([[0, -1, 0, 10], [1, 0, 0, 20],
                                    [0, 0, 1, 30], [0, 0, 0, 1]], dtype=float),
                "000001": np.array([[1, 0, 0, 1], [0, 1, 0, 2],
                                    [0, 0, 1, 3], [0, 0, 0, 1]], dtype=float),
            }
            for name, transform in transforms.items():
                submap = root / name
                submap.mkdir()
                (submap / "points_compact.bin").write_bytes(
                    np.array([[1, 0, 0]], dtype="<f4").tobytes())
                matrix = "\n".join(" ".join(str(value) for value in row) for row in transform)
                (submap / "data.txt").write_text(
                    f"id: {name}\nT_world_origin:\n{matrix}\nT_origin_endpoint_L:\n")
            cloud, submaps = EXPORT.load_glim_dump(root)
            self.assertEqual([path.name for path in submaps], ["000001", "000002"])
            np.testing.assert_allclose(cloud, [[2, 2, 3], [10, 21, 30]])

    def test_resampling_preserves_explicit_split_and_direction(self):
        trajectory = np.array([[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0], [0, 0, 0]], dtype=float)
        manifest = EXPORT.make_route_manifest(
            trajectory, 2, 1.0, "1" * 64, "2" * 64, "3" * 64, 0.1)
        outbound = manifest["outbound_route"]["waypoints"]
        returning = manifest["return_route"]["waypoints"]
        self.assertEqual((outbound[-1]["x_m"], outbound[-1]["y_m"]), (2.0, 2.0))
        self.assertEqual((returning[0]["x_m"], returning[0]["y_m"]), (2.0, 2.0))
        self.assertEqual((returning[-1]["x_m"], returning[-1]["y_m"]), (0.0, 0.0))
        self.assertAlmostEqual(returning[0]["yaw_rad"], np.pi)
        self.assertFalse(manifest["provenance"]["surveyed"])
        self.assertEqual(manifest["status"], "candidate")

    def test_recorded_corridor_is_cleared_after_inflation_without_erasing_adjacent_obstacle(self):
        trajectory = np.array([[0, 0, 0], [4, 0, 0]], dtype=float)
        cloud = np.array([[2, 0, 0.7], [2, 2, 0.7]], dtype=float)
        grid, origin, _, occupied, radius, cleared = EXPORT.build_grid(
            cloud, trajectory, resolution=1.0, padding=1.0, obstacle_min=0.15,
            obstacle_max=2.0, projection_mode="ground-relative",
            footprint_width=2.0, footprint_length=2.0, margin=0.0)
        inside = np.floor((np.array([2.0, 0.0]) - origin) / 1.0).astype(int)
        outside = np.floor((np.array([2.0, 2.0]) - origin) / 1.0).astype(int)
        self.assertEqual(occupied, 2)
        self.assertEqual(radius, 1)
        self.assertGreater(cleared, 0)
        self.assertEqual(grid[inside[1], inside[0]], EXPORT.FREE)
        self.assertEqual(grid[outside[1], outside[0]], EXPORT.OCCUPIED)

    def test_grid_handles_empty_ground_and_obstacle_masks_deterministically(self):
        trajectory = np.array([[0, 0, 0], [2, 0, 0]], dtype=float)
        kwargs = dict(
            resolution=1.0, padding=1.0, obstacle_min=0.15, obstacle_max=2.0,
            projection_mode="ground-relative", footprint_width=0.1,
            footprint_length=0.1, margin=0.0)

        numpy_unique = np.unique

        def reject_empty_cell_unique(values, *args, **kwargs):
            if values.shape == (0, 2) and kwargs.get("axis") == 0:
                raise AssertionError("empty cell selections must not reach np.unique")
            return numpy_unique(values, *args, **kwargs)

        with mock.patch.object(EXPORT.np, "unique", side_effect=reject_empty_cell_unique):
            obstacle_only = EXPORT.build_grid(
                np.array([[1, 0, 0.7]], dtype=float), trajectory, **kwargs)
            ground_only = EXPORT.build_grid(
                np.array([[1, 0, 0.0]], dtype=float), trajectory, **kwargs)
        obstacle_only_repeat = EXPORT.build_grid(
            np.array([[1, 0, 0.7]], dtype=float), trajectory, **kwargs)

        self.assertEqual(obstacle_only[0].shape, (3, 5))
        self.assertEqual(obstacle_only[2].shape[1], 2)
        self.assertEqual(obstacle_only[3], 1)
        np.testing.assert_array_equal(obstacle_only[0], obstacle_only_repeat[0])
        np.testing.assert_array_equal(obstacle_only[2], obstacle_only_repeat[2])
        self.assertEqual(ground_only[0].shape, (3, 5))
        self.assertEqual(ground_only[2].shape[1], 2)
        self.assertEqual(ground_only[3], 0)

    def test_export_is_byte_reproducible_and_has_trinary_semantics(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cloud_path, trajectory_path = root / "cloud.npy", root / "trajectory.npy"
            trajectory = np.array([[0, 0, 0], [2, 0, 0], [2, 2, .2], [0, 2, .2], [0, 0, 0]], dtype=float)
            ground = np.column_stack((np.linspace(0, 2, 30), np.zeros(30), np.zeros(30)))
            cloud = np.vstack((ground, trajectory, [[1.0, 1.0, 0.7]]))
            np.save(cloud_path, cloud)
            np.save(trajectory_path, trajectory)
            arguments = argparse.Namespace(
                cloud=str(cloud_path), trajectory=str(trajectory_path), trajectory_has_time=False,
                output_dir=str(root / "out"), map_name="map", route_name="routes", resolution=0.2,
                padding=1.0, gravity=(0.0, 0.0, -1.0), projection_mode="ground-relative",
                obstacle_min_height=0.15, obstacle_max_height=2.0, footprint_source="simulation",
                footprint_width=0.1, footprint_length=0.1, clearance_margin=0.0, route_spacing=0.5,
                split_index=2, grade_min_distance=0.2, safety_manifest_sha256="0" * 64)
            first = EXPORT.export(arguments)
            first_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (root / "out").iterdir()}
            second = EXPORT.export(arguments)
            second_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (root / "out").iterdir()}
            self.assertEqual(first_hashes, second_hashes)
            self.assertEqual(first["grid"]["semantics"], {"occupied": 0, "unknown": 205, "free": 254})
            self.assertGreaterEqual(first["grid"]["cleared_occupied_cells_in_recorded_corridor"], 0)
            self.assertEqual(first["hashes"], second["hashes"])
            route = json.loads((root / "out" / "routes.yaml").read_text())
            self.assertEqual(route["outbound_route"]["direction"], "outbound")
            self.assertEqual(route["return_route"]["direction"], "return")

    def test_route_map_obstacle_misalignment_is_blocking(self):
        grid = np.full((10, 10), EXPORT.FREE, dtype=np.uint8)
        grid[5, 5] = EXPORT.OCCUPIED
        trajectory = np.array([[0, 0, 0], [5, 5, 0], [9, 9, 0]], dtype=float)
        manifest = EXPORT.make_route_manifest(
            trajectory, 1, 1.0, "1" * 64, "2" * 64, "3" * 64, 0.0)
        errors = EXPORT.check_route_alignment(manifest, grid, np.array([0.0, 0.0]), 1.0, 0.0)
        self.assertTrue(any("lacks" in item for item in errors))


if __name__ == "__main__":
    unittest.main()
