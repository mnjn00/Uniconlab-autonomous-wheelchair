"""Timestamped, motion-compensated point-cloud accumulation."""

from __future__ import annotations

import math
import threading

import numpy as np
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft

from body_frame import body_to_lidar
from obstacle_cluster_geometry import interpolate_rigid_pose


WINDOW_S = 0.6


class Accumulator:
    """Short scan history motion-compensated into the newest body frame."""

    def __init__(
            self, lidar_in_body, lidar_to_body_rotation,
            odom_frame="camera_init", body_frame="body", cloud_frame="body"):
        self.lidar_in_body = lidar_in_body
        self.lidar_to_body_rotation = lidar_to_body_rotation
        self.odom_frame = str(odom_frame)
        self.body_frame = str(body_frame)
        self.cloud_frame = str(cloud_frame)
        self.lock = threading.RLock()
        self.scans = []
        self.odoms = []
        self.reference = None
        self.odom_valid = self.cloud_valid = True
        self.generation = 0

    def add_odom(self, message):
        with self.lock:
            self._add_odom(message)

    def _add_odom(self, message):
        try:
            q = message.pose.pose.orientation
            p = message.pose.pose.position
            stamp = message.header.stamp.to_sec()
            frame_id = str(message.header.frame_id)
            child_frame_id = str(message.child_frame_id)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._invalidate_odom()
            return
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not all(math.isfinite(v) for v in
                   (stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or \
                stamp <= 0.0 or abs(norm - 1.0) > 0.05 or \
                frame_id != self.odom_frame or \
                child_frame_id != self.body_frame or \
                (self.odoms and stamp <= self.odoms[-1][0]):
            self._invalidate_odom()
            return
        transform = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        transform[:3, 3] = (p.x, p.y, p.z)
        history = tuple(self.odoms)
        self.odoms = list((history + ((stamp, transform),))[-80:])
        self.reference = None
        self.odom_valid = True
        self.generation += 1

    def _invalidate_odom(self) -> None:
        self.odoms = []
        self.reference = None
        self.odom_valid = False
        self.generation += 1

    def nearest(self, stamp):
        with self.lock:
            return None if not self.odom_valid else interpolate_rigid_pose(
                self.odoms, stamp)

    def add_cloud(self, message):
        with self.lock:
            self._add_cloud(message)

    def _add_cloud(self, message):
        try:
            stamp = message.header.stamp.to_sec()
            frame_id = str(message.header.frame_id)
            points = np.array(list(pc2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True)),
                dtype=np.float32)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._invalidate_cloud()
            return
        if not math.isfinite(stamp) or stamp <= 0.0 or not len(points) or \
                points.ndim != 2 or points.shape[1:] != (3,) or \
                not np.isfinite(points).all() or frame_id != self.cloud_frame \
                or (self.scans and stamp <= self.scans[-1][0]):
            self._invalidate_cloud()
            return
        history = tuple(self.scans) + ((stamp, points),)
        self.scans = [item for item in history
                      if stamp - item[0] <= WINDOW_S + 0.3]
        self.reference = None
        self.cloud_valid = True
        self.generation += 1

    def _invalidate_cloud(self) -> None:
        self.scans = []
        self.reference = None
        self.cloud_valid = False
        self.generation += 1

    def merged(self):
        with self.lock:
            return self._merged()

    def _merged(self):
        self.reference = None
        generation = self.generation
        scans, odoms = tuple(self.scans), tuple(self.odoms)
        if not scans or not odoms or not self.cloud_valid \
                or not self.odom_valid:
            return None
        newest = scans[-1][0]
        reference = interpolate_rigid_pose(odoms, newest)
        if reference is None:
            return None
        inverse = np.linalg.inv(reference)
        parts = []
        for stamp, points in scans:
            if newest - stamp > WINDOW_S:
                continue
            transform = interpolate_rigid_pose(odoms, stamp)
            if transform is None:
                continue
            relative = (inverse @ transform).astype(np.float32)
            parts.append(
                np.dot(points, relative[:3, :3].T) + relative[:3, 3])
        if not parts:
            return None
        self.reference = (newest, reference)
        merged = body_to_lidar(
            np.vstack(parts), self.lidar_in_body,
            self.lidar_to_body_rotation)
        if generation != self.generation or not self.cloud_valid \
                or not self.odom_valid:
            self.reference = None
            return None
        return merged
