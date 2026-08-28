#!/usr/bin/env python3
"""Turn a planned nav_msgs/Path into the follower's route + safety-band assets.

The follower (waypoint_follower.py / dwa_follower.py) loads a route JSON and a
safety-band JSON at startup. This tool takes the global path navfn returned
over the drop-safe costmap and writes both, so the existing DWA follower can
drive an automatically planned route with no code change to its loading path.

The path is in the map frame and is a chair-CENTRE trajectory (the costmap it
was planned over was the eroded configuration space), so the route is emitted
with reference_point "chair_centre" and the mount's own chair_centre_in_body_xyz
- the same provenance the recorded routes carry. Height is sampled from the
ground surface the costmap baker wrote, because the safety band needs a per-
station height to locate ground against the 3D map.

The safety band is regenerated along the NEW path by the existing
make_route_safety_band.py, so the runtime drop protection covers ground the
planner chose rather than only the recorded line. That is the second layer of
the defence in depth: the costmap refused the drop at plan time, and the band
refuses it again at run time.

Usage
-----
    path_to_route_assets.py <path.json> --costmap <dropsafe.npz> \\
        --out-route <route.json> --out-band-prefix <band-prefix> \\
        [--map-pcd <map.pcd>] [--body-frame-profile builtin] \\
        [--step 0.2] [--chair-centre -0.5,-0.2,0.0]

<path.json> is a list of {"x":..,"y":..} or {"pose":{"position":{"x":..,"y":..}}}
as written by make_plan_client.py from a nav_msgs/Path.

Nothing here touches the runtime graph or grants motion authority.
"""

import argparse
import json
import math
import os
import subprocess
import sys

import numpy as np


def load_path(path_json):
    """Accept the flat or nav_msgs/Path-shaped JSON make_plan_client writes."""
    data = json.load(open(path_json, encoding="utf-8"))
    if isinstance(data, dict) and "poses" in data:
        data = data["poses"]
    pts = []
    for item in data:
        if "pose" in item and "position" in item["pose"]:
            p = item["pose"]["position"]
            pts.append([float(p["x"]), float(p["y"])])
        else:
            pts.append([float(item["x"]), float(item["y"])])
    return np.array(pts, dtype=float)


def resample(polyline, step_m):
    """Even arc-length resampling, the same scheme make_global_plan uses."""
    delta = np.diff(polyline, axis=0)
    leg = np.hypot(delta[:, 0], delta[:, 1])
    arc = np.concatenate([[0.0], np.cumsum(leg)])
    if arc[-1] <= 0.0:
        return polyline.copy()
    wanted = np.arange(0.0, arc[-1] + step_m * 0.5, step_m)
    return np.column_stack([np.interp(wanted, arc, polyline[:, 0]),
                            np.interp(wanted, arc, polyline[:, 1])])


def tangent_yaw_deg(polyline):
    """Per-point heading from the chord, smoothed at the ends."""
    n = len(polyline)
    yaw = np.zeros(n)
    for i in range(n):
        a = max(i - 1, 0)
        b = min(i + 1, n - 1)
        dx = polyline[b, 0] - polyline[a, 0]
        dy = polyline[b, 1] - polyline[a, 1]
        yaw[i] = math.degrees(math.atan2(dy, dx))
    return yaw


def sample_ground_height(points, costmap_npz):
    """Per-point ground height from the baked costmap's surface, else 0.

    The band generator walks the 3D map's height relative to the route's own
    height at each station, so a planned path needs a height along it. The
    costmap baker already recovered the ground surface; sampling it here avoids
    a second pass over the raw cloud.
    """
    if costmap_npz is None:
        return np.zeros(len(points))
    grid = np.load(costmap_npz)
    cell = float(grid["cell"])
    min_x, min_y = float(grid["min_x"]), float(grid["min_y"])
    ground = grid["ground"]
    ny, nx = ground.shape
    rows = np.clip(np.rint((points[:, 1] - min_y) / cell - 0.5).astype(int),
                   0, ny - 1)
    cols = np.clip(np.rint((points[:, 0] - min_x) / cell - 0.5).astype(int),
                   0, nx - 1)
    h = ground[rows, cols]
    h = np.where(np.isfinite(h), h, 0.0)
    return h


def build_route_doc(dense, z, yaw_deg, body_frame_profile, chair_centre,
                    step_m, path_length_m):
    waypoints = []
    for i, (xy, zi, yi) in enumerate(zip(dense, z, yaw_deg)):
        waypoints.append({
            "x": round(float(xy[0]), 3),
            "y": round(float(xy[1]), 3),
            "z": round(float(zi), 3),
            "yaw_deg": round(float(yi), 2)})
    return {
        "frame": "map",
        "source": "navfn global plan over drop-safe costmap (auto-generated)",
        "body_frame_profile": body_frame_profile,
        "reference_point": "chair_centre",
        "chair_centre_in_body_xyz": list(chair_centre),
        "route_step_m": step_m,
        "count": len(waypoints),
        "path_length_m": round(path_length_m, 3),
        "waypoints": waypoints}


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert a planned path to follower route + band assets.")
    parser.add_argument("path_json", help="planned path JSON")
    parser.add_argument("--costmap", help="dropsafe.npz from the baker "
                        "(for ground height)")
    parser.add_argument("--out-route", required=True, help="route waypoints JSON")
    parser.add_argument("--out-band-prefix",
                        help="band output prefix; triggers band generation")
    parser.add_argument("--map-pcd", help="3D map .pcd (required for the band)")
    parser.add_argument("--body-frame-profile", default="builtin",
                        help="the profile the follower runs (default builtin)")
    parser.add_argument("--step", type=float, default=0.2,
                        help="waypoint spacing in metres (default %(default).1f)")
    parser.add_argument("--chair-centre", default="-0.5,-0.2,0.0",
                        help="chair_centre_in_body_xyz (default %(default)s)")
    args = parser.parse_args(argv)

    chair_centre = [float(v) for v in args.chair_centre.split(",")]

    raw = load_path(args.path_json)
    if len(raw) < 2:
        sys.exit("path has fewer than 2 points; nothing to resample")
    dense = resample(raw, args.step)
    yaw = tangent_yaw_deg(dense)
    z = sample_ground_height(dense, args.costmap)
    length = float(np.hypot(*np.diff(dense, axis=0).T).sum())

    route = build_route_doc(dense, z, yaw, args.body_frame_profile,
                            chair_centre, args.step, length)
    with open(args.out_route, "w", encoding="utf-8") as f:
        json.dump(route, f, indent=1, ensure_ascii=False)
    print("route: %d waypoints, %.1f m, step %.2f m -> %s"
          % (len(dense), length, args.step, args.out_route), flush=True)

    if args.out_band_prefix:
        if not args.map_pcd:
            sys.exit("--map-pcd is required to generate the safety band")
        band_tool = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "make_route_safety_band.py")
        cmd = [sys.executable, band_tool, args.map_pcd, args.out_route,
               args.out_band_prefix]
        print("generating band: %s" % " ".join(cmd), flush=True)
        subprocess.run(cmd, check=True)
        print("band -> %s.json" % args.out_band_prefix, flush=True)


if __name__ == "__main__":
    main()
