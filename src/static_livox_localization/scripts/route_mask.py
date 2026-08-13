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
        distance = float(np.linalg.norm(end - start))
        count = max(2, int(np.ceil(distance / (self.resolution * 0.5))) + 1)
        t = np.linspace(0.0, 1.0, count)[:, None]
        return bool(self.contains_many(start + (end - start) * t).all())

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
