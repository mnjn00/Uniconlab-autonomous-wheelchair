"""Lightweight AB3DMOT-style tracking for clustered LiDAR obstacles.

The detector in :mod:`obstacle_clusters` already produces 3-D boxes. This
module supplies the part AB3DMOT is responsible for: constant-velocity
Kalman prediction, globally consistent Hungarian data association, stable
identities, and bounded survival through short occlusions.

Tracking is deliberately performed in the odom frame. A parked object moves
in the lidar frame whenever the chair moves, so a body-frame filter would
estimate ego motion as object motion. The producer owns the timestamped odom
transform and converts detections before calling this module.

This is a small runtime adaptation of the AB3DMOT design, not a vendored copy
of its KITTI/nuScenes evaluation stack. It uses the NumPy/SciPy dependencies
already required by this ROS package and accepts the legacy ``(x, y, label)``
input as well as an optional opaque payload used by the producer.
"""

import math

import numpy as np
from scipy.optimize import linear_sum_assignment


MOVING = "moving"
STATIC = "static"
UNKNOWN = "unknown"

# The Mahalanobis gate grows with prediction uncertainty; this Euclidean
# ceiling prevents an uncertain track from claiming another object far away.
ASSOCIATION_GATE_M = 1.2
MAX_ASSOCIATION_DISTANCE_M = 2.0
MAHALANOBIS_GATE_SQ = 9.21       # chi-square, 2 DoF, 99 percent

# The producer runs at 5 Hz. Keeping a track for 1.5 s bridges a turn or a
# short cluster dropout without preserving a ghost indefinitely.
DROP_AFTER_S = 1.5
CONFIRM_S = 1.5
STATIC_SPEED_MPS = 0.20

MEASUREMENT_STD_M = 0.20
ACCELERATION_STD_MPS2 = 1.50
INITIAL_POSITION_STD_M = 0.50
INITIAL_VELOCITY_STD_MPS = 2.00
LABEL_HISTORY_S = 2.0


def _finite(value, default=0.0):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float(default)
    return result if math.isfinite(result) else float(default)


def _detection(item):
    """Return ``(x, y, label, payload)`` from a supported detection."""
    if isinstance(item, dict):
        return (_finite(item.get("x")), _finite(item.get("y")),
                str(item.get("label", "obstacle")), item.get("payload"))
    if len(item) == 3:
        x, y, label = item
        payload = None
    elif len(item) == 4:
        x, y, label, payload = item
    else:
        raise ValueError("detections must contain x, y, label[, payload]")
    return _finite(x), _finite(y), str(label), payload


def _payload_size(payload):
    if not isinstance(payload, dict):
        return None
    try:
        size = np.asarray(payload["size"], dtype=np.float64).reshape(-1)
    except (KeyError, TypeError, ValueError):
        return None
    if len(size) < 2 or not np.isfinite(size[:2]).all():
        return None
    return np.maximum(size[:2], 0.1)


class Track(object):
    """One constant-velocity Kalman track in the odom frame."""

    def __init__(self, track_id, label, x, y, stamp_s, payload=None):
        self.id = int(track_id)
        self.state = np.array([x, y, 0.0, 0.0], dtype=np.float64)
        self.covariance = np.diag([
            INITIAL_POSITION_STD_M ** 2,
            INITIAL_POSITION_STD_M ** 2,
            INITIAL_VELOCITY_STD_MPS ** 2,
            INITIAL_VELOCITY_STD_MPS ** 2,
        ])
        self.first_seen = float(stamp_s)
        self.last_seen = float(stamp_s)
        self.last_prediction = float(stamp_s)
        self.hits = 1
        self.misses = 0
        self.payload = payload
        self.label_history = [(float(stamp_s), str(label))]
        self.label = str(label)

    @property
    def position(self):
        return float(self.state[0]), float(self.state[1])

    @property
    def velocity(self):
        return float(self.state[2]), float(self.state[3])

    def age_s(self, now_s):
        return max(0.0, float(now_s) - self.first_seen)

    def miss_age_s(self, now_s):
        return max(0.0, float(now_s) - self.last_seen)

    @property
    def predicted_only(self):
        return self.misses > 0

    def speed_mps(self):
        return math.hypot(*self.velocity)

    def uncertainty_m(self):
        values = np.linalg.eigvalsh(self.covariance[:2, :2])
        return math.sqrt(max(0.0, float(values[-1])))

    def motion(self, now_s, confirm_s=CONFIRM_S,
               static_speed_mps=STATIC_SPEED_MPS):
        if self.age_s(now_s) < confirm_s:
            return UNKNOWN
        return STATIC if self.speed_mps() < static_speed_mps else MOVING

    def _select_label(self):
        counts = {}
        for _stamp, label in self.label_history:
            counts[label] = counts.get(label, 0) + 1
        if not counts:
            return self.label
        highest = max(counts.values())
        tied = {label for label, count in counts.items() if count == highest}
        # A one-frame footprint fluctuation must not make a confirmed person
        # disappear. Person wins a tie, then the most recent tied label wins.
        if "person" in tied:
            return "person"
        for _stamp, label in reversed(self.label_history):
            if label in tied:
                return label
        return self.label

    def predict(self, stamp_s):
        stamp_s = float(stamp_s)
        dt = stamp_s - self.last_prediction
        if dt <= 0.0:
            return
        # Bound a clock jump. Track lifetime is governed independently by
        # last_seen; integrating an arbitrary pause would launch a stale
        # velocity estimate through the scene before that check removes it.
        dt = min(dt, DROP_AFTER_S)
        transition = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        q = ACCELERATION_STD_MPS2 ** 2
        process = q * np.array([
            [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
            [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
            [dt3 / 2.0, 0.0, dt2, 0.0],
            [0.0, dt3 / 2.0, 0.0, dt2],
        ])
        self.state = transition.dot(self.state)
        self.covariance = transition.dot(self.covariance).dot(
            transition.T) + process
        self.covariance = (self.covariance + self.covariance.T) * 0.5
        self.last_prediction = stamp_s
        self.misses += 1

    def innovation(self, x, y):
        residual = np.array([x, y], dtype=np.float64) - self.state[:2]
        innovation_covariance = self.covariance[:2, :2] + \
            np.eye(2) * MEASUREMENT_STD_M ** 2
        return residual, innovation_covariance

    def observe(self, label, x, y, stamp_s, payload=None):
        residual, innovation_covariance = self.innovation(x, y)
        gain = self.covariance[:, :2].dot(
            np.linalg.inv(innovation_covariance))
        self.state = self.state + gain.dot(residual)
        self.covariance = self.covariance - gain.dot(
            self.covariance[:2, :])
        self.covariance = (self.covariance + self.covariance.T) * 0.5
        self.last_seen = float(stamp_s)
        self.last_prediction = float(stamp_s)
        self.hits += 1
        self.misses = 0
        self.payload = payload
        self.label_history.append((float(stamp_s), str(label)))
        self.label_history = [
            value for value in self.label_history
            if float(stamp_s) - value[0] <= LABEL_HISTORY_S
        ]
        self.label = self._select_label()


class Tracker(object):
    """AB3DMOT-style prediction, association, update, birth and death."""

    def __init__(self, gate_m=ASSOCIATION_GATE_M,
                 drop_after_s=DROP_AFTER_S,
                 max_association_distance_m=MAX_ASSOCIATION_DISTANCE_M):
        self.gate_m = float(gate_m)
        self.drop_after_s = float(drop_after_s)
        self.max_association_distance_m = float(max_association_distance_m)
        self.tracks = []
        self.next_id = 1

    def update(self, detections, stamp_s):
        """Update tracks and return one track per detection, in input order.

        Unmatched live tracks remain available through :meth:`coasting` so
        the producer can publish predicted obstacles during a short dropout.
        """
        stamp_s = float(stamp_s)
        parsed = [_detection(item) for item in detections]
        self.tracks = [
            track for track in self.tracks
            if stamp_s - track.last_seen <= self.drop_after_s
        ]
        for track in self.tracks:
            track.predict(stamp_s)

        assigned_detections = {}
        if parsed and self.tracks:
            invalid = 1.0e9
            costs = np.full((len(parsed), len(self.tracks)), invalid,
                            dtype=np.float64)
            positions = np.asarray(
                [[item[0], item[1]] for item in parsed], dtype=np.float64)
            # Invert one 2x2 innovation covariance per track, not once for
            # every detection/track pair.  Forty clusters otherwise caused
            # 1,600 tiny Python/BLAS calls in every 5 Hz cycle on the NUC.
            for t_index, track in enumerate(self.tracks):
                residuals = positions - track.state[:2]
                innovation_covariance = track.covariance[:2, :2] + \
                    np.eye(2) * MEASUREMENT_STD_M ** 2
                inverse = np.linalg.inv(innovation_covariance)
                mahalanobis = np.einsum(
                    "ij,jk,ik->i", residuals, inverse, residuals)
                euclidean = np.linalg.norm(residuals, axis=1)
                expanded_gate = min(
                    self.max_association_distance_m,
                    self.gate_m + track.miss_age_s(stamp_s),
                )
                valid = (euclidean <= expanded_gate) & \
                        (mahalanobis <= MAHALANOBIS_GATE_SQ)
                costs[valid, t_index] = mahalanobis[valid]

                previous_size = _payload_size(track.payload)
                for d_index in np.flatnonzero(valid).tolist():
                    _x, _y, label, payload = parsed[d_index]
                    current_size = _payload_size(payload)
                    if current_size is not None and previous_size is not None:
                        costs[d_index, t_index] += 0.15 * float(np.abs(
                            np.log(current_size / previous_size)).sum())
                    # Classification is evidence, never a hard gate. The
                    # footprint heuristic can flicker during a turn.
                    if label != track.label:
                        costs[d_index, t_index] += 0.25
            rows, columns = linear_sum_assignment(costs)
            for d_index, t_index in zip(rows.tolist(), columns.tolist()):
                if costs[d_index, t_index] < invalid:
                    assigned_detections[d_index] = t_index

        result = []
        for d_index, (x, y, label, payload) in enumerate(parsed):
            if d_index in assigned_detections:
                track = self.tracks[assigned_detections[d_index]]
                track.observe(label, x, y, stamp_s, payload)
            else:
                track = Track(self.next_id, label, x, y, stamp_s, payload)
                self.next_id += 1
                self.tracks.append(track)
            result.append(track)
        return result

    def coasting(self, stamp_s):
        """Tracks predicted at ``stamp_s`` but not measured in that cycle."""
        stamp_s = float(stamp_s)
        return [
            track for track in self.tracks
            if track.predicted_only and
            track.miss_age_s(stamp_s) <= self.drop_after_s
        ]
