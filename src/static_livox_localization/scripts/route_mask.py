"""Authoritative drivable-mask containment and boundary cost."""

import os

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

FREE = 254
BOUNDARY_FREE_M = 0.5


class RouteMask:
    """Immutable raster geometry used by the rollout planner."""

    def __init__(self, yaml_path):
        with open(yaml_path, encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        image_path = os.path.join(
            os.path.dirname(os.path.abspath(yaml_path)), metadata["image"])
        self.free = np.asarray(Image.open(image_path)) == FREE
        self.resolution = float(metadata["resolution"])
        self.origin = np.asarray(metadata["origin"][:2], dtype=float)
        self.clearance = ndimage.distance_transform_edt(
            self.free) * self.resolution

    def _cells(self, points):
        array = np.asarray(points, dtype=float)
        col = np.rint(
            (array[:, 0] - self.origin[0]) / self.resolution).astype(int)
        row = np.rint(
            self.free.shape[0] - 1
            - (array[:, 1] - self.origin[1]) / self.resolution).astype(int)
        valid = (
            (row >= 0) & (row < self.free.shape[0])
            & (col >= 0) & (col < self.free.shape[1]))
        return row, col, valid

    def contains_many(self, points):
        """Whether every point's chair centre lies in the drawn region."""
        row, col, valid = self._cells(points)
        contained = np.zeros(len(row), dtype=bool)
        contained[valid] = self.free[row[valid], col[valid]]
        return contained

    def contains(self, point):
        """Whether one chair-centre point lies in the authoritative mask."""
        return bool(self.contains_many([point])[0])

    def segment_is_contained(self, start, end):
        """Check every raster cell crossed by a chair-centre segment."""
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        start_grid = np.array([
            (start[0] - self.origin[0]) / self.resolution,
            self.free.shape[0] - 1
            - (start[1] - self.origin[1]) / self.resolution,
        ])
        end_grid = np.array([
            (end[0] - self.origin[0]) / self.resolution,
            self.free.shape[0] - 1
            - (end[1] - self.origin[1]) / self.resolution,
        ])
        delta = end_grid - start_grid
        crossings = [0.0, 1.0]
        for axis in range(2):
            if abs(delta[axis]) < 1e-12:
                continue
            low, high = sorted((start_grid[axis], end_grid[axis]))
            first = int(np.floor(low - 0.5)) + 1
            last = int(np.ceil(high - 0.5))
            for cell in range(first, last + 1):
                boundary = cell + 0.5
                t = (boundary - start_grid[axis]) / delta[axis]
                if 0.0 < t < 1.0:
                    crossings.append(float(t))
        crossings = np.unique(np.asarray(crossings))
        probes = np.concatenate((
            crossings,
            (crossings[:-1] + crossings[1:]) * 0.5,
        ))
        points = start + probes[:, None] * (end - start)
        if not self.contains_many(points).all():
            return False
        # A segment exactly through a raster corner touches both orthogonal
        # cells. Rounding sees only one diagonal cell; the supercover must
        # reject either forbidden neighbour as well.
        for t in crossings[1:-1]:
            grid = start_grid + t * delta
            on_x = abs(grid[0] - (round(grid[0] - 0.5) + 0.5)) < 1e-10
            on_y = abs(grid[1] - (round(grid[1] - 0.5) + 0.5)) < 1e-10
            if on_x and on_y:
                rows = [
                    int(np.floor(grid[1])),
                    int(np.ceil(grid[1])),
                ]
                cols = [
                    int(np.floor(grid[0])),
                    int(np.ceil(grid[0])),
                ]
                if any(
                    not (
                        0 <= row < self.free.shape[0]
                        and 0 <= col < self.free.shape[1]
                        and self.free[row, col]
                    )
                    for row in rows
                    for col in cols
                ):
                    return False
        return True

    def paths_are_contained(self, paths):
        paths = np.asarray(paths, dtype=float)
        result = np.ones(len(paths), dtype=bool)
        for index, path in enumerate(paths):
            result[index] = all(
                self.segment_is_contained(start, end)
                for start, end in zip(path[:-1], path[1:])
            )
        return result

    def boundary_cost_many(self, points):
        """Zero with room to spare, rising quadratically to one at the edge."""
        row, col, valid = self._cells(points)
        clearance = np.zeros(len(row), dtype=float)
        clearance[valid] = self.clearance[row[valid], col[valid]]
        normalized = np.clip(
            (BOUNDARY_FREE_M - clearance) / BOUNDARY_FREE_M, 0.0, 1.0)
        cost = np.square(normalized)
        cost[~self.contains_many(points)] = np.inf
        return cost
