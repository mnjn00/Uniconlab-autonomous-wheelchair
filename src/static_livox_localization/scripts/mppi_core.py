"""Sampling-based MPPI local planner for the field wheelchair stack.

The planner is intentionally a drop-in replacement for ``DwaPlanner.plan``:
``plan(state, obstacles, speed_cap, last_yaw_rate, last_speed)`` returns a
single executable target ``(v, w, status)`` while the existing follower keeps
all motion guards, semantic WAIT/GO_ROUND policy, command ramp and watchdogs.

This first branch is a bench/replay controller, not a field-authorized default.
It preserves the DWA safety contracts:
- route mask is the hard physical boundary;
- leaving the preferred band is expensive but only allowed when the route mask
  independently proves the rollout drivable;
- obstacle clearance below the same 0.50 m floor is a hard reject;
- the loaded-chair forward deadband is represented by target commands of 0 or
  >= 0.35 m/s;
- GPU failure is fail-closed when GPU execution is required.

CuPy is used for stochastic control sampling and rollout integration. The
route/band/mask geometry remains on the exact CPU implementations already
validated by the DWA stack; obstacle nearest-neighbour scoring reuses the
existing RTX/CuPy distance backend.
"""

from __future__ import annotations

import math
import os

import numpy as np

import dwa_core
from array_backend import resolve
from gpu_dwa_backend import DwaDistanceBackend, GpuRequiredError

# Conservative first-pass defaults for the 10 Hz field loop.
HORIZON_STEPS = 30
MODEL_DT = 0.10
BATCH_SIZE = 384
TEMPERATURE = 0.70
NOISE_V = 0.18
NOISE_W = 0.22
SEED = 42

# Cost terms deliberately begin close to the measured DWA trade-offs.
W_PATH = dwa_core.W_PATH
W_ROUTE_DEVIATION = dwa_core.W_ROUTE_DEVIATION
W_HEADING = dwa_core.W_HEADING
W_PROGRESS = dwa_core.W_PROGRESS
W_OBSTACLE = dwa_core.W_OBSTACLE
W_CENTRE = dwa_core.W_CENTRE
W_MASK_BOUNDARY = dwa_core.W_MASK_BOUNDARY
BAND_ESCAPE_BASE_COST = dwa_core.BAND_ESCAPE_BASE_COST
W_BAND_OVERFLOW = dwa_core.W_BAND_OVERFLOW
W_VELOCITY = dwa_core.W_VELOCITY
W_STEER_RATE = 0.50
W_SPEED_RATE = 0.20
W_TERMINAL_PATH = 2.0


class MppiPlanner(object):
    """Receding-horizon MPPI planner with the DWA field safety contracts."""

    def __init__(self, band, route, route_mask=None, max_speed=dwa_core.MAX_SPEED,
                 horizon_steps=HORIZON_STEPS, model_dt=MODEL_DT,
                 batch_size=BATCH_SIZE, temperature=TEMPERATURE,
                 noise_v=NOISE_V, noise_w=NOISE_W, seed=SEED,
                 grace=0.0, prefer_gpu=True, require_gpu=False, log=None):
        from scipy.spatial import cKDTree

        self.band = band
        self.route = np.asarray(route, dtype=np.float64)
        if self.route.ndim != 2 or self.route.shape[1] != 2 or len(self.route) < 2:
            raise ValueError("route must be an Nx2 array with at least two points")
        self.route_mask = route_mask
        self.max_speed = float(max_speed)
        self.steps = max(2, int(horizon_steps))
        self.dt = max(0.02, float(model_dt))
        self.batch_size = max(32, int(batch_size))
        self.temperature = max(1e-4, float(temperature))
        self.noise_v = max(1e-4, float(noise_v))
        self.noise_w = max(1e-4, float(noise_w))
        self.grace = float(grace)
        self.log = log or (lambda message: None)
        self.seed = int(seed)

        self.tree = cKDTree(self.route)
        seg = np.linalg.norm(np.diff(self.route, axis=0), axis=1)
        self.arc = np.concatenate([[0.0], np.cumsum(seg)])
        tangent = np.gradient(self.route, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        self.heading = np.arctan2(tangent[:, 1], tangent[:, 0])

        self.backend = resolve(prefer_gpu=bool(prefer_gpu), log=self.log)
        if require_gpu and not self.backend.on_gpu:
            raise GpuRequiredError(
                "MPPI GPU required but CuPy is unavailable: %s" %
                (self.backend.reason or "unknown"))
        self.require_gpu = bool(require_gpu)
        self.backend_name = self.backend.name
        self.distance_backend = DwaDistanceBackend(
            self.route, prefer_gpu=bool(prefer_gpu),
            require_gpu=bool(require_gpu), log=self.log)
        self.gpu_active = self.backend.on_gpu and self.distance_backend.on_gpu

        xp = self.backend.xp
        self.rng = xp.random.RandomState(self.seed)
        self.nominal = xp.zeros((self.steps, 2), dtype=xp.float32)
        self.initialized = False
        self.last_cost = float("inf")
        self.last_feasible = 0

    def arc_at(self, point):
        return float(self.arc[int(self.tree.query(np.asarray(point, dtype=float))[1])])

    @staticmethod
    def _clip_targets(xp, controls, cap):
        """Target commands are either stop or above the measured wheel floor."""
        out = controls.copy()
        v = xp.clip(out[:, :, 0], 0.0, float(cap))
        floor = float(dwa_core.TURN_FLOOR_SPEED)
        # A stop is a refusal returned by plan(), never a sampled trajectory.
        # This is the same lesson as the field DWA: a stationary rollout on
        # the route can score artificially well because its path error is zero.
        v = xp.maximum(v, floor)
        out[:, :, 0] = v
        out[:, :, 1] = xp.clip(
            out[:, :, 1], -float(dwa_core.MAX_YAW_RATE),
            float(dwa_core.MAX_YAW_RATE))
        return out

    def _seed_nominal(self, cap, last_speed, last_yaw_rate):
        held = 0.0 if last_speed is None else max(0.0, float(last_speed))
        if held < dwa_core.TURN_FLOOR_SPEED:
            held = min(float(cap), float(dwa_core.TURN_FLOOR_SPEED))
        else:
            held = min(float(cap), held)
        self.nominal[:, 0] = held
        self.nominal[:, 1] = float(last_yaw_rate)
        self.initialized = True

    def _sample_controls(self, cap):
        xp = self.backend.xp
        noise = self.rng.normal(
            0.0, 1.0, size=(self.batch_size, self.steps, 2)).astype(xp.float32)
        noise[:, :, 0] *= self.noise_v
        noise[:, :, 1] *= self.noise_w
        # Preserve the nominal trajectory as sample zero. This guarantees that
        # stochastic exploration cannot remove the previous feasible answer.
        noise[0, :, :] = 0.0
        controls = self.nominal[None, :, :] + noise
        controls = self._clip_targets(xp, controls, cap)
        return controls, noise

    def _rollout(self, state, controls):
        """Vectorized differential-drive rollout on the selected array backend."""
        xp = self.backend.xp
        state = np.asarray(state, dtype=float)
        x = xp.full(self.batch_size, float(state[0]), dtype=xp.float32)
        y = xp.full(self.batch_size, float(state[1]), dtype=xp.float32)
        yaw = xp.full(self.batch_size, float(state[2]), dtype=xp.float32)
        path = xp.empty((self.batch_size, self.steps, 3), dtype=xp.float32)
        for step in range(self.steps):
            v = controls[:, step, 0]
            w = controls[:, step, 1]
            # Midpoint heading reduces integration bias without changing the
            # simple differential-drive model used by the existing controller.
            yaw_mid = yaw + 0.5 * w * self.dt
            x = x + v * xp.cos(yaw_mid) * self.dt
            y = y + v * xp.sin(yaw_mid) * self.dt
            yaw = yaw + w * self.dt
            path[:, step, 0] = x
            path[:, step, 1] = y
            path[:, step, 2] = yaw
        return path

    def _obstacle_clearance(self, paths_host, obstacles):
        if not len(obstacles):
            return np.full(self.batch_size, np.inf, dtype=float)
        flat = paths_host[:, :, :2].reshape(-1, 2)
        distance, _ = self.distance_backend.obstacle_query(flat, obstacles)
        return distance.reshape(self.batch_size, self.steps).min(axis=1)

    def _geometric_cost(self, state, paths_host, controls_host, obstacles):
        flat = paths_host[:, :, :2].reshape(-1, 2)
        lateral, lo, hi = self.band.margins_many(flat)
        band_inside = self.band.contained(lateral, lo, hi, self.grace)
        band_inside_path = band_inside.reshape(self.batch_size, self.steps)

        if self.route_mask is None:
            feasible = band_inside_path.all(axis=1)
            mask_boundary = np.zeros(self.batch_size, dtype=float)
        else:
            feasible = self.route_mask.contains_many(flat).reshape(
                self.batch_size, self.steps).all(axis=1)
            feasible &= self.route_mask.paths_are_contained(paths_host[:, :, :2])
            mask_boundary = self.route_mask.boundary_cost_many(flat).reshape(
                self.batch_size, self.steps).mean(axis=1)

        clear = self._obstacle_clearance(paths_host, obstacles)
        feasible &= clear >= float(dwa_core.OBSTACLE_FLOOR_M)

        distance, index = self.distance_backend.route_query(flat)
        route_distance = distance.reshape(self.batch_size, self.steps)
        path_cost = route_distance.mean(axis=1)
        route_deviation = np.square(route_distance).mean(axis=1)

        ref_heading = self.heading[index].reshape(self.batch_size, self.steps)
        aim = np.abs(np.arctan2(
            np.sin(paths_host[:, :, 2] - ref_heading),
            np.cos(paths_host[:, :, 2] - ref_heading))).mean(axis=1)

        here = self.arc_at(state[:2])
        end_index = self.tree.query(paths_host[:, -1, :2])[1]
        progress = self.arc[end_index] - here
        obstacle_penalty = np.where(
            np.isfinite(clear), np.maximum(0.0, 1.0 - clear), 0.0)

        half = np.maximum((hi - lo) / 2.0, 1e-6)
        edge = np.abs(lateral - (hi + lo) / 2.0) / half
        centre = np.square(np.minimum(edge, 1.0)).reshape(
            self.batch_size, self.steps).mean(axis=1)

        overflow = (np.maximum(lo - lateral, 0.0) +
                    np.maximum(lateral - hi, 0.0))
        escaped = (~band_inside_path).any(axis=1)
        band_escape = (
            BAND_ESCAPE_BASE_COST * escaped.astype(float) +
            W_BAND_OVERFLOW * np.square(overflow).reshape(
                self.batch_size, self.steps).mean(axis=1))

        dv = np.diff(controls_host[:, :, 0], axis=1)
        dw = np.diff(controls_host[:, :, 1], axis=1)
        speed_rate = np.square(dv).mean(axis=1) if dv.shape[1] else 0.0
        steer_rate = np.square(dw).mean(axis=1) if dw.shape[1] else 0.0
        velocity_reward = controls_host[:, :, 0].mean(axis=1)
        terminal_path = route_distance[:, -1]

        cost = (
            W_PATH * path_cost +
            W_ROUTE_DEVIATION * route_deviation +
            W_HEADING * aim -
            W_PROGRESS * progress +
            W_OBSTACLE * obstacle_penalty +
            W_CENTRE * centre +
            band_escape +
            W_MASK_BOUNDARY * mask_boundary +
            W_SPEED_RATE * speed_rate +
            W_STEER_RATE * steer_rate +
            W_TERMINAL_PATH * terminal_path -
            W_VELOCITY * velocity_reward)
        return cost, feasible, clear

    def _update_nominal(self, controls, noise, cost, feasible, cap):
        xp = self.backend.xp
        feasible_idx = np.flatnonzero(feasible)
        if not len(feasible_idx):
            return False
        finite_cost = cost[feasible_idx]
        rho = float(np.min(finite_cost))
        scaled = -(finite_cost - rho) / self.temperature
        scaled = np.clip(scaled, -60.0, 0.0)
        weights_host = np.exp(scaled)
        total = float(weights_host.sum())
        if not math.isfinite(total) or total <= 1e-12:
            return False
        weights_host /= total

        idx = xp.asarray(feasible_idx, dtype=xp.int64)
        weights = xp.asarray(weights_host, dtype=xp.float32)
        selected_noise = noise[idx]
        delta = xp.sum(weights[:, None, None] * selected_noise, axis=0)
        self.nominal = self.nominal + delta
        nominal_batch = self.nominal[None, :, :]
        self.nominal = self._clip_targets(xp, nominal_batch, cap)[0]
        self.last_cost = float(np.sum(weights_host * finite_cost))
        self.last_feasible = int(len(feasible_idx))
        return True

    def plan(self, state, obstacles=(), speed_cap=None,
             last_yaw_rate=0.0, last_speed=None):
        cap = self.max_speed if speed_cap is None else min(
            self.max_speed, max(0.0, float(speed_cap)))
        if cap < float(dwa_core.TURN_FLOOR_SPEED):
            return 0.0, 0.0, "SPEED_BELOW_FLOOR"

        if not self.initialized:
            self._seed_nominal(cap, last_speed, last_yaw_rate)
        else:
            # A long hold/manual takeover can make the old receding-horizon
            # sequence irrelevant. Re-seed only on a large mismatch.
            held = 0.0 if last_speed is None else max(0.0, float(last_speed))
            first = float(self.backend.tohost(self.nominal[0, 0]))
            if abs(first - held) > 0.45:
                self._seed_nominal(cap, last_speed, last_yaw_rate)

        try:
            controls, noise = self._sample_controls(cap)
            paths = self._rollout(state, controls)
            paths_host = self.backend.tohost(paths).astype(np.float64, copy=False)
            controls_host = self.backend.tohost(controls).astype(np.float64, copy=False)
            cost, feasible, _ = self._geometric_cost(
                np.asarray(state, dtype=float), paths_host, controls_host,
                np.asarray(obstacles, dtype=float).reshape(-1, 2)
                if len(obstacles) else ())
            if not feasible.any():
                self.last_feasible = 0
                # Distinguish physical-boundary failure from obstacle failure
                # with one geometry-only retry of the feasibility predicates.
                flat = paths_host[:, :, :2].reshape(-1, 2)
                if self.route_mask is None:
                    lat, lo, hi = self.band.margins_many(flat)
                    physical = self.band.contained(lat, lo, hi, self.grace).reshape(
                        self.batch_size, self.steps).all(axis=1)
                else:
                    physical = self.route_mask.contains_many(flat).reshape(
                        self.batch_size, self.steps).all(axis=1)
                    physical &= self.route_mask.paths_are_contained(
                        paths_host[:, :, :2])
                return (0.0, 0.0,
                        "OBSTACLE" if physical.any() and len(obstacles)
                        else "OFF_BAND")
            if not self._update_nominal(controls, noise, cost, feasible, cap):
                return 0.0, 0.0, "NO_CANDIDATE"

            target = self.backend.tohost(self.nominal[0]).astype(float)
            target_v = float(target[0])
            target_w = float(target[1])

            # Receding horizon: consume one control and hold the tail value as
            # the next cycle's prior. Copy to avoid overlapping-view surprises
            # on either NumPy or CuPy.
            xp = self.backend.xp
            shifted = self.nominal.copy()
            shifted[:-1] = self.nominal[1:]
            shifted[-1] = self.nominal[-1]
            self.nominal = shifted
            if self.backend.on_gpu:
                xp.cuda.Device().synchronize()
            return target_v, target_w, "OK"
        except GpuRequiredError:
            return 0.0, 0.0, "GPU_ERROR"
        except Exception as error:  # fail closed; caller logs the status
            self.log("MPPI planner error: %s: %s" %
                     (type(error).__name__, error))
            if self.require_gpu:
                return 0.0, 0.0, "GPU_ERROR"
            return 0.0, 0.0, "PLANNER_ERROR"


def planner_from_environment(band, route, route_mask=None, log=None, **kwargs):
    prefer = os.environ.get("WHEELCHAIR_MPPI_GPU", "1") != "0"
    require = os.environ.get("WHEELCHAIR_REQUIRE_GPU", "0") == "1"
    kwargs.setdefault("prefer_gpu", prefer)
    kwargs.setdefault("require_gpu", require)
    kwargs.setdefault("log", log)
    return MppiPlanner(band, route, route_mask=route_mask, **kwargs)
