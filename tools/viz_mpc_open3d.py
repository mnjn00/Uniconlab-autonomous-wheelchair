#!/usr/bin/env python3
"""Open3D scene of the MPC follower's simulated runs on the route v4 band.

Renders what the planner actually sees and obeys - the safety band, the
route, and the obstacles - with the three simulated trajectories from
tools/sim_mpc_follower.py laid over it, so the planner's behaviour can be
eyeballed before it ever touches the chair:

  blue lines   band edges (the usable limits the chair centre must respect)
  grey ticks   band stations
  yellow dots  route v4 waypoints
  green        clear run
  orange       obstacle run (passes the wide-station obstacle in-band)
  red          blocked run (controlled stop short of the choke obstacle)
  magenta      obstacles with their 0.45 m padding rings
  boxes        chair footprint at start / goal / blocked stop

Generate the trajectories first:
    python3 tools/sim_mpc_follower.py --save-json /tmp/mpc_traj.json
then open the scene:
    python3 tools/viz_mpc_open3d.py --json /tmp/mpc_traj.json
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parents[1]

C_BG = [0.07, 0.07, 0.09]
C_EDGE = [0.30, 0.50, 0.90]
C_TICK = [0.30, 0.34, 0.44]
C_ROUTE = [0.90, 0.80, 0.25]
C_CLEAR = [0.25, 0.85, 0.35]
C_OBST = [1.00, 0.55, 0.05]
C_BLOCK = [0.95, 0.25, 0.35]
C_OBS = [0.95, 0.15, 0.60]
C_CHAIR = [0.85, 0.85, 0.90]


def polyline_lines(points, z, color):
    pts = np.column_stack([points[:, 0], points[:, 1],
                           np.full(len(points), z)])
    lines = np.column_stack([np.arange(len(pts) - 1), np.arange(1, len(pts))])
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines = o3d.utility.Vector2iVector(lines)
    ls.paint_uniform_color(color)
    return ls


def chair_box(x, y, yaw, color=C_CHAIR):
    """Chair footprint 0.7 m long x 0.6 m wide, centred on (x, y)."""
    box = o3d.geometry.TriangleMesh.create_box(
        width=0.7, height=0.6, depth=0.35)
    box.translate([-0.35, -0.30, 0.0])
    box.rotate(box.get_rotation_matrix_from_axis_angle([0, 0, yaw]),
               center=[0, 0, 0])
    box.translate([x, y, 0.02])
    box.paint_uniform_color(color)
    box.compute_vertex_normals()
    return box


def ring(x, y, r, color, z=0.01, n=64):
    th = np.linspace(0, 2 * math.pi, n)
    pts = np.column_stack([x + r * np.cos(th), y + r * np.sin(th),
                           np.full(n, z)])
    lines = np.column_stack([np.arange(n - 1), np.arange(1, n)])
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines = o3d.utility.Vector2iVector(lines)
    ls.paint_uniform_color(color)
    return ls


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--json", default="/tmp/mpc_traj.json",
                        help="trajectory dump from sim_mpc_follower.py "
                             "--save-json")
    args = parser.parse_args(argv)

    with open(ROOT / "routes" / "20260802_route_v4_waypoints.json") as f:
        waypoints = json.load(f)["waypoints"]
    with open(ROOT / "routes" / "20260802_route_v4_safety_band.json") as f:
        stations = json.load(f)["stations"]
    traj_data = json.load(open(args.json))

    sxy = np.array([[s["x"], s["y"]] for s in stations])
    h = np.radians([s["heading_deg"] for s in stations])
    normals = np.stack([-np.sin(h), np.cos(h)], axis=1)
    left = np.array([max(s["left_m"] - 0.45, 0.15) for s in stations])
    right = np.array([max(s["right_m"] - 0.45, 0.15) for s in stations])

    geoms = []
    geoms.append(polyline_lines(sxy + normals * left[None].T, 0.0, C_EDGE))
    geoms.append(polyline_lines(sxy - normals * right[None].T, 0.0, C_EDGE))
    ticks = []
    for i in range(0, len(sxy), 10):
        ticks.append([sxy[i] - normals[i] * right[i],
                      sxy[i] + normals[i] * left[i]])
    tick_lines = o3d.geometry.LineSet()
    tpts, tlines = [], []
    for a, b in ticks:
        tlines.append([len(tpts), len(tpts) + 1])
        tpts.append([a[0], a[1], 0.0])
        tpts.append([b[0], b[1], 0.0])
    tick_lines.points = o3d.utility.Vector3dVector(np.array(tpts))
    tick_lines.lines = o3d.utility.Vector2iVector(np.array(tlines))
    tick_lines.paint_uniform_color(C_TICK)
    geoms.append(tick_lines)

    wxy = np.array([[w["x"], w["y"]] for w in waypoints])[::8]
    route_pcd = o3d.geometry.PointCloud()
    route_pcd.points = o3d.utility.Vector3dVector(
        np.column_stack([wxy[:, 0], wxy[:, 1], np.full(len(wxy), 0.005)]))
    route_pcd.paint_uniform_color(C_ROUTE)
    geoms.append(route_pcd)

    colors = {"clear": C_CLEAR, "obstacle": C_OBST, "blocked": C_BLOCK}
    zlift = {"clear": 0.03, "obstacle": 0.06, "blocked": 0.09}
    stops = {}
    for name, scen in traj_data["scenarios"].items():
        t = np.array(scen["traj"])
        if len(t) < 2:
            continue
        geoms.append(polyline_lines(t[:, :2], zlift[name], colors[name]))
        if scen["final"] != "GOAL":
            stops[name] = t[-1]
            geoms.append(chair_box(t[-1, 0], t[-1, 1], t[-1, 2],
                                   colors[name]))

    for name, station in traj_data.get("obstacle_station", {}).items():
        if name not in traj_data["scenarios"]:
            continue
        o = sxy[station]
        ball = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
        ball.translate([o[0], o[1], 0.15])
        ball.paint_uniform_color(C_OBS)
        ball.compute_vertex_normals()
        geoms.append(ball)
        geoms.append(ring(o[0], o[1], 0.45, C_OBS))

    s = traj_data["start"]
    g = traj_data["goal"]
    geoms.append(chair_box(s[0], s[1], s[2], [0.2, 0.6, 0.9]))
    geoms.append(chair_box(g[0], g[1], 0.0, [0.2, 0.8, 0.4]))

    centre = sxy.mean(axis=0)
    extent = float(np.linalg.norm(sxy.max(axis=0) - sxy.min(axis=0)))
    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=2.0, origin=[sxy[0, 0], sxy[0, 1], 0]))

    print(__doc__)
    print(f"band stations: {len(sxy)}, scenarios: "
          f"{list(traj_data['scenarios'].keys())}")
    for name, scen in traj_data["scenarios"].items():
        print(f"  {name}: final={scen['final']}, steps={len(scen['traj'])}")
    o3d.visualization.draw_geometries(
        geoms, window_name="MPC follower - route v4 simulation",
        width=1280, height=900,
        front=[-0.30, -0.30, 0.85], lookat=[centre[0], centre[1], 0.0],
        up=[0.0, 0.0, 1.0], zoom=0.12 * (120.0 / max(extent, 1.0)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
