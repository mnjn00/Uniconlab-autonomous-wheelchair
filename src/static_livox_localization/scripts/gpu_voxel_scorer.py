"""GPU coarse scorer for the seedless initial-pose search.

The exact cKDTree metric remains the authority for refinement.  This module
only makes the coarse pass wide enough to bracket poses that are far from the
recorded mapping trajectory.  Map points are voxelised once, dilated by the
inlier radius, and candidate scans then become independent indexed lookups.
"""

import math

import numpy as np

from array_backend import resolve


DEFAULT_VOXEL_M = 0.45
DEFAULT_CHUNK = 1024
MAX_SUPPORT_KEYS = 100_000_000


class GpuVoxelUnavailable(RuntimeError):
    """The accelerator cannot safely construct or score this map."""


class GpuVoxelScorer(object):
    """A conservative binary map-support scorer on sorted sparse voxel keys."""

    def __init__(
        self,
        map_points,
        inlier_radius_m,
        voxel_size_m=DEFAULT_VOXEL_M,
        chunk=DEFAULT_CHUNK,
        log=None,
    ):
        self.log = log or (lambda message: None)
        self.backend = resolve(prefer_gpu=True, log=self.log)
        if not self.backend.on_gpu:
            raise GpuVoxelUnavailable(self.backend.reason)
        if voxel_size_m <= 0.0 or inlier_radius_m <= 0.0:
            raise ValueError("voxel size and inlier radius must be positive")
        self.xp = self.backend.xp
        self.voxel_size_m = float(voxel_size_m)
        self.chunk = max(int(chunk), 1)
        self._build(map_points, float(inlier_radius_m))

    def _build(self, map_points, radius):
        points = np.asarray(map_points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            raise ValueError("map_points must be a non-empty Nx3 array")

        # Padding keeps neighbour expansion and boundary queries simple.  The
        # deployed map's bounding box contains 1.3 billion 20 cm cells because
        # of sparse high/low returns, so a dense array is categorically wrong
        # for this map even though most of it is empty.
        cells = int(math.ceil(radius / self.voxel_size_m)) + 1
        padding = cells * self.voxel_size_m
        lower = points.min(axis=0) - padding
        upper = points.max(axis=0) + padding
        shape = (
            np.floor((upper - lower) / self.voxel_size_m).astype(np.int64) + 1
        )
        address_space = int(np.prod(shape, dtype=np.int64))
        if address_space <= 0 or address_space >= np.iinfo(np.int64).max:
            raise GpuVoxelUnavailable(
                "voxel key address space does not fit int64")

        xp = self.xp
        self.lower = xp.asarray(lower, dtype=xp.float32)
        self.shape = tuple(int(value) for value in shape)
        indices = xp.floor(
            (xp.asarray(points, dtype=xp.float32) - self.lower)
            / self.voxel_size_m
        ).astype(xp.int32)
        occupied_keys = xp.unique(self._encode(indices))
        iz = occupied_keys % self.shape[2]
        flat_xy = occupied_keys // self.shape[2]
        iy = flat_xy % self.shape[1]
        ix = flat_xy // self.shape[1]
        occupied_indices = xp.stack((ix, iy, iz), axis=1).astype(xp.int32)

        # Include a half-voxel quantisation allowance.  Coarse scoring is
        # allowed false positives (the exact refinement rejects them), but a
        # false negative here could remove the true pose before refinement.
        allowance = radius + math.sqrt(3.0) * self.voxel_size_m * 0.5
        neighbour_offsets = []
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                for dz in range(-cells, cells + 1):
                    distance = (
                        math.sqrt(dx * dx + dy * dy + dz * dz)
                        * self.voxel_size_m
                    )
                    if distance <= allowance:
                        neighbour_offsets.append((dx, dy, dz))
        estimated = len(occupied_keys) * len(neighbour_offsets)
        if estimated > MAX_SUPPORT_KEYS:
            raise GpuVoxelUnavailable(
                "sparse support would create up to {} keys (limit {})".format(
                    int(estimated), MAX_SUPPORT_KEYS))

        expanded = []
        bounds = xp.asarray(self.shape, dtype=xp.int32)
        for offset in neighbour_offsets:
            shifted = occupied_indices + xp.asarray(offset, dtype=xp.int32)
            valid = xp.all((shifted >= 0) & (shifted < bounds), axis=1)
            expanded.append(self._encode(shifted[valid]))
        self.support_keys = xp.unique(xp.concatenate(expanded))
        support_count = int(self.support_keys.size)
        del indices, occupied_keys, occupied_indices, expanded
        xp.cuda.Device().synchronize()
        self.log(
            "GPU sparse voxel map: {}x{}x{} address space, {:.1f}M support "
            "keys, voxel {:.2f} m".format(
                self.shape[0], self.shape[1], self.shape[2],
                support_count / 1.0e6, self.voxel_size_m))

    def _encode(self, indices):
        indices = indices.astype(self.xp.int64, copy=False)
        return ((indices[:, 0] * self.shape[1] + indices[:, 1])
                * self.shape[2] + indices[:, 2])

    def score_poses(self, sample, poses):
        """Return map-support fractions in input pose order.

        ``poses`` contains x, y, z, yaw.  Transform and lookup both remain on
        the device; only one score per hypothesis crosses back to the host.
        """

        if not poses:
            return np.empty(0, np.float64)
        xp = self.xp
        sample_gpu = xp.asarray(sample, dtype=xp.float32)
        scores = np.empty(len(poses), np.float64)
        shape = xp.asarray(self.shape, dtype=xp.int32)
        for start in range(0, len(poses), self.chunk):
            host = np.asarray(poses[start:start + self.chunk], dtype=np.float32)
            batch = xp.asarray(host)
            cosine = xp.cos(batch[:, 3])[:, None]
            sine = xp.sin(batch[:, 3])[:, None]
            sx = sample_gpu[None, :, 0]
            sy = sample_gpu[None, :, 1]
            world_x = cosine * sx - sine * sy + batch[:, 0, None]
            world_y = sine * sx + cosine * sy + batch[:, 1, None]
            world_z = sample_gpu[None, :, 2] + batch[:, 2, None]
            ix = xp.floor(
                (world_x - self.lower[0]) / self.voxel_size_m
            ).astype(xp.int32)
            iy = xp.floor(
                (world_y - self.lower[1]) / self.voxel_size_m
            ).astype(xp.int32)
            iz = xp.floor(
                (world_z - self.lower[2]) / self.voxel_size_m
            ).astype(xp.int32)
            valid = (
                (ix >= 0) & (ix < shape[0])
                & (iy >= 0) & (iy < shape[1])
                & (iz >= 0) & (iz < shape[2])
            )
            query_indices = xp.stack((ix, iy, iz), axis=2)
            query_keys = self._encode(query_indices.reshape(-1, 3))
            locations = xp.searchsorted(self.support_keys, query_keys)
            inside = locations < len(self.support_keys)
            safe_locations = xp.minimum(locations, len(self.support_keys) - 1)
            found = (
                inside
                & (self.support_keys[safe_locations] == query_keys)
                & valid.reshape(-1)
            ).reshape(valid.shape)
            block_scores = found.mean(axis=1, dtype=xp.float64)
            scores[start:start + len(host)] = self.backend.tohost(block_scores)
        return scores
