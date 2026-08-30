"""CuPy acceleration for the heavy obstacle-clearance part of DWA.

The route critic deliberately remains on the existing CPU ``cKDTree``. A
closed route can contain duplicate or crossing points; changing nearest-index
tie-breaking changes route heading and progress even when the distance is the
same. The GPU is therefore used where the target NUC actually needs it and
where the result is unambiguous: thousands of extended rollout samples against
all current obstacle samples.

The CPU path remains the numerical fallback. A hybrid field launch may set
``~require_gpu:=true``; then a missing or failed CUDA device produces a planner
hold instead of silently returning to the high-load CPU obstacle query.
"""

from __future__ import annotations

import math
import os

import numpy as np
from scipy.spatial import cKDTree

from array_backend import resolve


class GpuRequiredError(RuntimeError):
    """GPU execution was required but cannot provide a valid answer."""


def _as_bool(value, default=False):
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    return bool(default)


def _ros_option(name, default):
    try:
        import rospy
        if rospy.core.is_initialized():
            return rospy.get_param("~" + name, default)
    except Exception:
        pass
    return default


def _ros_log(message):
    try:
        import rospy
        if rospy.core.is_initialized():
            rospy.loginfo(message)
    except Exception:
        pass


class DwaDistanceBackend(object):
    """RTX obstacle queries with an exact CPU route-query contract."""

    def __init__(self, route_points, prefer_gpu=True, require_gpu=False,
                 log=None, query_chunk=1024, reference_chunk=2048):
        route = np.asarray(route_points, dtype=np.float64)
        if route.ndim != 2 or route.shape[1] != 2 or not len(route):
            raise ValueError("route_points must be a non-empty Nx2 array")
        if not np.isfinite(route).all():
            raise ValueError("route_points must be finite")
        self.log = log or _ros_log
        self.require_gpu = bool(require_gpu)
        self.query_chunk = max(1, int(query_chunk))
        self.reference_chunk = max(1, int(reference_chunk))
        self.route_host = route
        self.route_tree = cKDTree(route)
        self.backend = resolve(prefer_gpu=bool(prefer_gpu), log=self.log)
        self.backend_name = self.backend.name
        self.failure_reason = ""
        if self.require_gpu and not self.backend.on_gpu:
            raise GpuRequiredError(
                "RTX/CuPy DWA backend unavailable: %s" %
                (self.backend.reason or "unknown"))

    @property
    def on_gpu(self):
        return self.backend_name == "cupy"

    def _gpu_failed(self, error):
        self.failure_reason = "%s: %s" % (type(error).__name__, error)
        self.backend_name = "numpy"
        self.log("DWA GPU backend failed; CPU fallback: %s" % self.failure_reason)
        if self.require_gpu:
            raise GpuRequiredError(self.failure_reason)

    @staticmethod
    def _validated(points, name):
        array = np.asarray(points, dtype=np.float32)
        if array.size == 0:
            return np.empty((0, 2), dtype=np.float32)
        array = array.reshape(-1, 2)
        if not np.isfinite(array).all():
            raise ValueError("%s must contain finite xy points" % name)
        return array

    def _gpu_query(self, query_points, reference_device):
        query = self._validated(query_points, "query_points")
        if not len(query):
            return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.int64)
        xp = self.backend.xp
        distances = np.empty(len(query), dtype=np.float64)
        indices = np.empty(len(query), dtype=np.int64)
        reference_count = int(reference_device.shape[0])
        if reference_count <= 0:
            distances.fill(np.inf)
            indices.fill(-1)
            return distances, indices

        for q_start in range(0, len(query), self.query_chunk):
            q_host = query[q_start:q_start + self.query_chunk]
            q = xp.asarray(q_host, dtype=xp.float32)
            best_sq = xp.full(len(q_host), xp.inf, dtype=xp.float32)
            best_index = xp.full(len(q_host), -1, dtype=xp.int64)
            rows = xp.arange(len(q_host), dtype=xp.int64)
            for r_start in range(0, reference_count, self.reference_chunk):
                reference = reference_device[
                    r_start:r_start + self.reference_chunk]
                dx = q[:, None, 0] - reference[None, :, 0]
                dy = q[:, None, 1] - reference[None, :, 1]
                squared = dx * dx + dy * dy
                local_index = xp.argmin(squared, axis=1)
                local_sq = squared[rows, local_index]
                better = local_sq < best_sq
                best_sq = xp.where(better, local_sq, best_sq)
                best_index = xp.where(
                    better, local_index.astype(xp.int64) + r_start,
                    best_index)
            count = len(q_host)
            distances[q_start:q_start + count] = np.sqrt(
                self.backend.tohost(best_sq).astype(np.float64))
            indices[q_start:q_start + count] = self.backend.tohost(
                best_index).astype(np.int64)
        return distances, indices

    def route_query(self, query_points):
        # Intentionally CPU: preserve cKDTree's existing duplicate/crossing
        # route-index behavior exactly. Only obstacle clearance is offloaded.
        query = np.asarray(query_points, dtype=np.float64).reshape(-1, 2)
        if not np.isfinite(query).all():
            raise ValueError("route query must contain finite xy points")
        return self.route_tree.query(query, workers=-1)

    def obstacle_query(self, query_points, obstacle_points):
        query = self._validated(query_points, "obstacle query")
        obstacles = self._validated(obstacle_points, "obstacle_points")
        if not len(query):
            return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.int64)
        if not len(obstacles):
            return (np.full(len(query), np.inf, dtype=np.float64),
                    np.full(len(query), -1, dtype=np.int64))
        if self.on_gpu:
            try:
                obstacle_device = self.backend.asarray(obstacles)
                result = self._gpu_query(query, obstacle_device)
                self.backend.xp.cuda.Device().synchronize()
                return result
            except Exception as error:
                self._gpu_failed(error)
        return cKDTree(obstacles).query(query, workers=-1)


def make_gpu_planner(base_class, core_module):
    """Return a ``DwaPlanner`` subclass using :class:`DwaDistanceBackend`."""

    class GpuDwaPlanner(base_class):
        def __init__(self, *args, **kwargs):
            explicit_prefer = kwargs.pop("prefer_gpu", None)
            explicit_require = kwargs.pop("require_gpu", None)
            explicit_log = kwargs.pop("gpu_log", None)
            super(GpuDwaPlanner, self).__init__(*args, **kwargs)
            prefer_default = _as_bool(
                os.environ.get("WHEELCHAIR_DWA_GPU", "1"), True)
            require_default = _as_bool(
                os.environ.get("WHEELCHAIR_REQUIRE_GPU", "0"), False)
            prefer_gpu = _as_bool(
                explicit_prefer if explicit_prefer is not None else
                _ros_option("prefer_gpu", prefer_default), prefer_default)
            require_gpu = _as_bool(
                explicit_require if explicit_require is not None else
                _ros_option("require_gpu", require_default), require_default)
            self.distance_backend = DwaDistanceBackend(
                self.route, prefer_gpu=prefer_gpu, require_gpu=require_gpu,
                log=explicit_log or _ros_log)
            self.distance_backend_name = self.distance_backend.backend_name
            self._publish_backend_state()

        def _publish_backend_state(self):
            self.distance_backend_name = self.distance_backend.backend_name
            try:
                import rospy
                if rospy.core.is_initialized():
                    rospy.set_param("~distance_backend",
                                    self.distance_backend_name)
                    rospy.set_param("~gpu_active",
                                    self.distance_backend.on_gpu)
            except Exception:
                pass

        def plan(self, state, obstacles=(), speed_cap=None,
                 last_yaw_rate=0.0, last_speed=None,
                 obstacle_floor_m=core_module.OBSTACLE_FLOOR_M,
                 actuator_state=None, committed_side=None, proposal_seq=None,
                 stamp_s=None, permit_track_id=None, return_proposal=False,
                 minimum_turn_rps=0.0):
            metadata = None
            try:
                side = core_module.normalize_side(committed_side)
                turn_floor = core_module.normalize_minimum_turn(
                    minimum_turn_rps)
                if return_proposal:
                    if not isinstance(
                            actuator_state, core_module.ActuatorState):
                        raise core_module.ProposalValidationError(
                            "actuator_state is invalid")
                    metadata = core_module.ProposalMetadata(
                        proposal_seq, stamp_s, permit_track_id, side)
                elif actuator_state is not None and not isinstance(
                        actuator_state, core_module.ActuatorState):
                    raise core_module.ProposalValidationError(
                        "actuator_state is invalid")
            except core_module.ProposalValidationError:
                if return_proposal:
                    return 0.0, 0.0, "ACTUATOR_STATE_INVALID", None
                raise
            cap = self.max_speed if speed_cap is None else min(
                self.max_speed, float(speed_cap))
            pairs = [
                (v, w)
                for v in core_module.speed_samples(cap, current=last_speed)
                if v > 0.0
                for w in core_module.yaw_samples()
                if (core_module.yaw_matches_side(side, w)
                    and abs(float(w)) + 1e-12 >= turn_floor)
            ]
            self.last_candidate_count = len(pairs)
            if not pairs:
                result = (0.0, 0.0, "SPEED_BELOW_FLOOR")
                return result + (None,) if return_proposal else result

            span = self.preview_distance(last_speed)
            pairs, paths, proposal_paths, applied_speeds, \
                applied_yaw_rates, travelled = \
                self._candidate_rollouts(
                    np.asarray(state, dtype=float), pairs, span,
                    actuator_state)
            self.last_candidate_count = len(pairs)
            path_steps = paths.shape[1]
            flat = paths[:, :, :2].reshape(-1, 2)
            lateral, lo, hi = self.band.margins_many(flat)
            band_inside = self.band.contained(lateral, lo, hi, self.grace)
            if self.route_mask is None:
                ok = band_inside.reshape(len(pairs), path_steps).all(axis=1)
            else:
                ok = self.route_mask.contains_many(flat).reshape(
                    len(pairs), path_steps).all(axis=1)
                ok &= self.route_mask.paths_are_contained(paths[:, :, :2])
            if not ok.any():
                result = (0.0, 0.0, "OFF_BAND")
                return result + (None,) if return_proposal else result

            try:
                if len(obstacles):
                    points = np.asarray(obstacles, dtype=float).reshape(-1, 2)
                    watched = self._obstacle_paths(
                        proposal_paths, travelled,
                        core_module.OBSTACLE_PREVIEW_M)
                    flat_watched = watched[:, :, :2].reshape(-1, 2)
                    distance, _ = self.distance_backend.obstacle_query(
                        flat_watched, points)
                    clear = distance.reshape(len(pairs), -1).min(axis=1)
                else:
                    clear = np.full(len(pairs), np.inf)
                ok &= clear >= float(obstacle_floor_m)
                if not ok.any():
                    result = (0.0, 0.0, "OBSTACLE")
                    return result + (None,) if return_proposal else result
                # Route lookup stays on the exact reference cKDTree contract.
                distance, index = self.distance_backend.route_query(flat)
                self._publish_backend_state()
            except GpuRequiredError:
                self._publish_backend_state()
                result = (0.0, 0.0, "GPU_ERROR")
                return result + (None,) if return_proposal else result

            route_distance = distance.reshape(len(pairs), path_steps)
            path_cost = route_distance.mean(axis=1)
            route_deviation = np.square(route_distance).mean(axis=1)
            here = self.arc_at(state[:2])
            ends = self.tree.query(paths[:, -1, :2])[1]
            progress = self.arc[ends] - here
            penalty = np.where(
                np.isfinite(clear), np.maximum(0.0, 1.0 - clear), 0.0)
            reference_heading = self.heading[index].reshape(
                len(pairs), path_steps)
            aim = np.abs(np.arctan2(
                np.sin(paths[:, :, 2] - reference_heading),
                np.cos(paths[:, :, 2] - reference_heading))).mean(axis=1)
            steer = np.abs(
                np.asarray([pair[1] for pair in pairs]) -
                float(last_yaw_rate))
            held = 0.0 if last_speed is None else float(last_speed)
            speed_change = np.abs(
                np.asarray([pair[0] for pair in pairs]) - held)
            half = np.maximum((hi - lo) / 2.0, 1e-6)
            edge = np.abs(lateral - (hi + lo) / 2.0) / half
            centre = np.square(np.minimum(edge, 1.0)).reshape(
                len(pairs), path_steps).mean(axis=1)
            overflow = (np.maximum(lo - lateral, 0.0) +
                        np.maximum(lateral - hi, 0.0))
            escaped = (~band_inside).reshape(
                len(pairs), path_steps).any(axis=1)
            band_escape = (
                core_module.BAND_ESCAPE_BASE_COST * escaped.astype(float) +
                core_module.W_BAND_OVERFLOW * np.square(overflow).reshape(
                    len(pairs), path_steps).mean(axis=1))
            if self.route_mask is None:
                mask_boundary = np.zeros(len(pairs), dtype=float)
            else:
                mask_boundary = self.route_mask.boundary_cost_many(flat).reshape(
                    len(pairs), path_steps).mean(axis=1)
            cost = (
                core_module.W_SPEED * speed_change -
                core_module.W_VELOCITY * np.asarray(
                    [pair[0] for pair in pairs]) +
                core_module.W_PATH * path_cost +
                core_module.W_ROUTE_DEVIATION * route_deviation +
                core_module.W_HEADING * aim -
                core_module.W_PROGRESS * progress +
                core_module.W_OBSTACLE * penalty +
                core_module.W_STEER * steer +
                core_module.W_CENTRE * centre + band_escape +
                core_module.W_MASK_BOUNDARY * mask_boundary)
            cost = np.where(ok, cost, np.inf)
            best = int(np.argmin(cost))
            selected = None
            if return_proposal:
                selected = core_module.TrajectoryProposal(
                    proposal_seq=metadata.proposal_seq,
                    stamp_s=metadata.stamp_s,
                    permit_track_id=metadata.permit_track_id,
                    committed_side=metadata.committed_side,
                    frame_id="body",
                    horizon_s=(len(proposal_paths[best])
                               * actuator_state.control_step_s),
                    actuator_state=actuator_state,
                    target_speed_mps=pairs[best][0],
                    target_yaw_rate_rps=pairs[best][1],
                    poses=self._body_relative(
                        proposal_paths[best], np.asarray(state, dtype=float)),
                    speeds_mps=applied_speeds[best],
                    yaw_rates_rps=applied_yaw_rates[best],
                )
            result = (float(pairs[best][0]), float(pairs[best][1]), "OK")
            return result + (selected,) if return_proposal else result

    GpuDwaPlanner.__name__ = "GpuDwaPlanner"
    return GpuDwaPlanner


def install_gpu_planner(dwa_core_module):
    """Replace ``dwa_core.DwaPlanner`` before ``DwaFollower`` constructs it."""
    planner = make_gpu_planner(dwa_core_module.DwaPlanner, dwa_core_module)
    dwa_core_module.DwaPlanner = planner
    return planner
