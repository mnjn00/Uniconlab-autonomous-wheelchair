#!/usr/bin/env python3
"""Closed-loop simulation of the MPC follower on the shipped route, no ROS.

Drives a simulated unicycle chair with mpc_core over the route v4 safety
band, exactly as docs/mpc_follower_design.md section 10 specifies: a clear
corridor run and a static-obstacle run. Reports the design's offline gates
(band containment, cross-track, solve time, silent-failure count) so the
planner is judged on evidence before it ever touches the chair.

Usage:
    python3 tools/sim_mpc_follower.py                # both scenarios
    python3 tools/sim_mpc_follower.py --scenario clear
    python3 tools/sim_mpc_follower.py --scenario obstacle

The plant model is the same unicycle the controller assumes, inputs applied
directly each 0.1 s step (no actuator lag yet - the CarND-style latency
compensation is untestable without the hardware).
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "static_livox_localization" / "scripts"))

import importlib.util


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"
core = _load("mpc_core", SCRIPTS / "mpc_core.py")
safety_band = _load("safety_band", SCRIPTS / "safety_band.py")

GOAL_TOLERANCE_M = 1.0
MAX_SIM_STEPS = 9000   # 15 minutes of simulated driving (route / cruise + margin)


def load_route(root):
    wp_path = root / "routes" / "20260802_route_v4_waypoints.json"
    band_path = root / "routes" / "20260802_route_v4_safety_band.json"
    with open(wp_path) as f:
        waypoints = json.load(f)["waypoints"]
    band = safety_band.SafetyBand(str(band_path))
    return waypoints, band


def run_scenario(name, band, goal_xy, start_x0, obstacles):
    ref = core.Reference(band)
    solver = core.MpcSolver(ref)
    x = start_x0.copy()
    warm = None
    traj, statuses, laterals, solve_ms = [], [], [], []
    wall_start = time.monotonic()
    steps = 0
    reached = False
    while steps < MAX_SIM_STEPS:
        if np.linalg.norm(x[:2] - goal_xy) < GOAL_TOLERANCE_M:
            reached = True
            break
        v_ref, th_ref = core.polyline_refs(band, x[:2], solver.p.horizon,
                                           solver.p.dt, 0.6)
        obs = []
        for o_xy in obstacles:
            if np.linalg.norm(o_xy - x[:2]) < solver.p.obstacle_plan_m + 4.0:
                obs.append(core.obstacle_half_plane(
                    ref, o_xy, solver.p.obstacle_padding))
        u0, status, info = solver.solve_cycle(x, v_ref, th_ref, obs, warm=warm)
        statuses.append(status)
        _n, _h, lat, lo, hi = ref.frame_at(x[:2])
        laterals.append((lat, lo, hi))
        solve_ms.append(info.get("solve_ms", 0.0))
        if status in (core.STATUS_INFEASIBLE_STOP, core.STATUS_BUDGET_STOP,
                      core.STATUS_BLOCKED_STOP):
            break  # the ladder stopped the chair; the scenario is over
        x = core.unicycle_step(x, u0, solver.p.dt)
        traj.append(x.copy())
        warm = (info.get("xbar"), info.get("ubar")) \
            if status == core.STATUS_OK else warm
        steps += 1
    wall = time.monotonic() - wall_start

    traj = np.array(traj) if traj else np.empty((0, 5))
    statuses = np.array(statuses)
    lat_arr = np.array([l for l, _lo, _hi in laterals])
    lo_arr = np.array([lo for _l, lo, _hi in laterals])
    hi_arr = np.array([hi for _l, _lo, hi in laterals])
    inside = (lat_arr >= lo_arr - 1e-6) & (lat_arr <= hi_arr + 1e-6)
    solve = np.array(solve_ms)
    ok = int((statuses == core.STATUS_OK).sum())

    print(f"--- scenario: {name}")
    print(f"    steps            : {steps} ({steps * 0.1:.0f} s simulated, "
          f"{wall:.1f} s wall, {steps * 0.1 / max(wall, 1e-9):.1f}x realtime)")
    print(f"    goal reached     : {reached}")
    if len(traj):
        print(f"    distance driven  : "
              f"{np.linalg.norm(np.diff(traj[:, :2], axis=0), axis=1).sum():.1f} m")
    print(f"    status counts    : "
          f"{ {s: int((statuses == s).sum()) for s in np.unique(statuses)} if len(statuses) else {} }")
    print(f"    band containment : {inside.mean() * 100 if len(inside) else 0:.2f}% "
          f"of {len(inside)} steps")
    if len(lat_arr):
        print(f"    |lateral|        : median {np.median(np.abs(lat_arr)):.3f} m, "
              f"p95 {np.percentile(np.abs(lat_arr), 95):.3f} m, "
              f"max {np.abs(lat_arr).max():.3f} m")
    if len(solve):
        print(f"    solve time       : p50 {np.percentile(solve, 50):.1f} ms, "
              f"p95 {np.percentile(solve, 95):.1f} ms, "
              f"p99 {np.percentile(solve, 99):.1f} ms, max {solve.max():.1f} ms")
    dmin = float("inf")
    if len(traj) and len(obstacles):
        dmin = min(np.linalg.norm(traj[:, :2] - o, axis=1).min()
                   for o in obstacles)
        print(f"    min obstacle dist: {dmin:.3f} m (padding "
              f"{solver.p.obstacle_padding:.2f} m)")
    final = "GOAL" if reached else (
        str(statuses[-1]) if len(statuses) else "TIMEOUT")
    return {"reached": reached, "containment": float(inside.mean())
            if len(inside) else 0.0, "steps": steps, "ok": ok,
            "min_obstacle_dist": dmin, "final": final,
            "traj": np.round(traj, 4).tolist() if len(traj) else [],
            "statuses": statuses.tolist()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scenario",
                        choices=["clear", "obstacle", "blocked", "both"],
                        default="both")
    parser.add_argument("--save-json", default=None,
                        help="write the simulated trajectories to this path "
                             "for tools/viz_mpc_open3d.py")
    args = parser.parse_args(argv)

    waypoints, band = load_route(ROOT)
    w0 = waypoints[0]
    yaw0 = np.radians(w0.get("yaw_deg", 0.0))
    start = np.array([w0["x"], w0["y"], yaw0, 0.0, 0.0])
    goal = np.array([waypoints[-1]["x"], waypoints[-1]["y"]])
    print(f"route v4: {len(waypoints)} waypoints, {len(band.xy)} band stations, "
          f"goal {goal[0]:.1f},{goal[1]:.1f}")

    # Scenario placement is part of the test: the obstacle goes where the
    # band says there IS room to pass, and the blocked scenario goes where
    # the band says there is not - a 0.5 m padded obstacle inside a 0.3 m
    # corridor is physically impassable and the correct answer is a
    # controlled stop, not a squeeze.
    widths = band.left + band.right
    n = len(band.xy)
    mid_range = slice(int(0.3 * n), int(0.7 * n))
    wide = int(mid_range.start + np.argmax(widths[mid_range]))
    narrow = int(mid_range.start + np.argmin(widths[mid_range]))
    print(f"obstacle at station {wide} (usable width {widths[wide]:.2f} m), "
          f"blocked at station {narrow} (usable width {widths[narrow]:.2f} m)")

    results = {}
    if args.scenario in ("clear", "both"):
        results["clear"] = run_scenario("clear", band, goal, start, [])
    if args.scenario in ("obstacle", "both"):
        results["obstacle"] = run_scenario(
            "obstacle", band, goal, start, [np.array(band.xy[wide])])
    if args.scenario in ("blocked", "both"):
        results["blocked"] = run_scenario(
            "blocked", band, goal, start, [np.array(band.xy[narrow])])

    print("=" * 70)
    print("GATES (docs/mpc_follower_design.md section 10)")
    clear = results.get("clear")
    if clear:
        print(f"  [{'PASS' if clear['containment'] == 1.0 else 'FAIL'}] "
              f"band containment 100%: {clear['containment'] * 100:.2f}%")
        print(f"  [{'PASS' if clear['reached'] else 'FAIL'}] "
              f"goal reached end to end")
    obs = results.get("obstacle")
    if obs:
        print(f"  [{'PASS' if obs['containment'] == 1.0 else 'FAIL'}] "
              f"obstacle run containment 100%: {obs['containment'] * 100:.2f}%")
        print(f"  [{'PASS' if obs['reached'] else 'FAIL'}] "
              f"obstacle run still reaches the goal")
    blk = results.get("blocked")
    if blk:
        print(f"  [{'PASS' if blk['containment'] == 1.0 else 'FAIL'}] "
              f"blocked run containment 100%: {blk['containment'] * 100:.2f}%")
        print(f"  [{'PASS' if not blk['reached'] else 'FAIL'}] "
              f"blocked run stops instead of squeezing through")
        print(f"  [{'PASS' if blk['min_obstacle_dist'] >= 0.35 else 'FAIL'}] "
              f"blocked run keeps >= 0.35 m from the obstacle: "
              f"{blk['min_obstacle_dist']:.3f} m")
    if args.save_json:
        payload = {"goal": [float(goal[0]), float(goal[1])],
                   "start": [float(start[0]), float(start[1]),
                             float(start[2])],
                   "obstacle_station": {"obstacle": int(wide),
                                        "blocked": int(narrow)},
                   "scenarios": results}
        with open(args.save_json, "w") as f:
            json.dump(payload, f)
        print(f"trajectories written to {args.save_json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
