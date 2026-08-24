#!/usr/bin/env python3
"""Render a top-down campus backdrop from the localization map, in the map frame.

The operator app draws the route and the chair's pose in the ``map`` frame. To
put a picture underneath, the picture has to be in that frame too -- and the one
image guaranteed to be is the one rendered from the point cloud the localizer is
matching against. A photo would need control points and an eyeballed fit; this
needs neither, because every pixel's map coordinate is arithmetic.

Structures come out of the height channel: a cell whose points reach well above
the local ground is a wall, a building face or a parked truck, and that is
exactly the context a first-time visitor needs to recognise where they are.

Writes <out>.png and <out>.json; the JSON carries the origin and resolution the
app needs to place the image.
"""

import argparse
import json
import os
import struct

import numpy as np
from PIL import Image


def read_pcd_xyz(path, max_points=None):
    """Binary PCD with float32 x y z intensity -- the format the map is stored in."""
    with open(path, "rb") as handle:
        fields, size, type_, count, points, data = None, None, None, None, None, None
        while True:
            line = handle.readline()
            if not line:
                raise ValueError("%s ended before DATA" % path)
            text = line.decode("ascii", "replace").strip()
            key, _, value = text.partition(" ")
            key = key.upper()
            if key == "FIELDS":
                fields = value.split()
            elif key == "SIZE":
                size = [int(v) for v in value.split()]
            elif key == "TYPE":
                type_ = value.split()
            elif key == "COUNT":
                count = [int(v) for v in value.split()]
            elif key == "POINTS":
                points = int(value)
            elif key == "DATA":
                data = value.strip().lower()
                break
        if data != "binary":
            raise ValueError("%s is DATA %s; only binary is handled" % (path, data))
        if type_ != ["F"] * len(fields) or size != [4] * len(fields) \
                or count != [1] * len(fields):
            raise ValueError("%s is not all float32 scalars" % path)
        raw = np.frombuffer(handle.read(points * 4 * len(fields)), dtype=np.float32)
    cloud = raw.reshape(-1, len(fields))
    idx = {name: i for i, name in enumerate(fields)}
    xyz = cloud[:, [idx["x"], idx["y"], idx["z"]]]
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if max_points and len(xyz) > max_points:
        step = len(xyz) // max_points + 1
        xyz = xyz[::step]
    return xyz


def render(xyz, resolution, bounds, ground_pct, wall_m):
    min_x, min_y, max_x, max_y = bounds
    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    inside = ((xyz[:, 0] >= min_x) & (xyz[:, 0] < max_x)
              & (xyz[:, 1] >= min_y) & (xyz[:, 1] < max_y))
    xyz = xyz[inside]
    if len(xyz) == 0:
        raise SystemExit("no points inside the requested bounds")

    col = ((xyz[:, 0] - min_x) / resolution).astype(np.int32)
    # Image row 0 is the top, which is max y.
    row = ((max_y - xyz[:, 1]) / resolution).astype(np.int32)
    np.clip(col, 0, width - 1, out=col)
    np.clip(row, 0, height - 1, out=row)
    flat = row.astype(np.int64) * width + col

    ground = float(np.percentile(xyz[:, 2], ground_pct))
    top = np.full(width * height, -np.inf, dtype=np.float32)
    np.maximum.at(top, flat, xyz[:, 2])
    hits = np.bincount(flat, minlength=width * height)

    top = top.reshape(height, width)
    hits = hits.reshape(height, width)
    seen = hits > 0
    relief = np.where(seen, top - ground, 0.0)

    # Three readable bands rather than a continuous ramp: nothing surveyed,
    # ground the chair could be on, and structure it certainly cannot.
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = (247, 249, 252)                      # unsurveyed
    flat_ground = seen & (relief < wall_m)
    image[flat_ground] = (223, 231, 238)            # ground
    structure = seen & (relief >= wall_m)
    tall = np.clip((relief - wall_m) / 6.0, 0.0, 1.0)
    image[structure] = np.stack([
        (150 - 60 * tall[structure]),
        (166 - 62 * tall[structure]),
        (184 - 60 * tall[structure]),
    ], axis=-1).astype(np.uint8)
    return Image.fromarray(image), ground, width, height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd")
    parser.add_argument("out", help="output path without extension")
    parser.add_argument("--route", help="waypoint JSON; bounds come from it")
    parser.add_argument("--margin-m", type=float, default=40.0)
    parser.add_argument("--resolution", type=float, default=0.25)
    parser.add_argument("--ground-percentile", type=float, default=25.0)
    parser.add_argument("--wall-m", type=float, default=1.2,
                        help="height above ground at which a cell counts as structure")
    parser.add_argument("--max-points", type=int, default=0)
    args = parser.parse_args()

    xyz = read_pcd_xyz(os.path.expanduser(args.pcd), args.max_points or None)
    print("cloud: %d points, x %.1f..%.1f  y %.1f..%.1f  z %.1f..%.1f"
          % (len(xyz), xyz[:, 0].min(), xyz[:, 0].max(),
             xyz[:, 1].min(), xyz[:, 1].max(), xyz[:, 2].min(), xyz[:, 2].max()))

    if args.route:
        with open(os.path.expanduser(args.route)) as handle:
            waypoints = json.load(handle).get("waypoints") or []
        rx = [float(w["x"]) for w in waypoints if w.get("x") is not None]
        ry = [float(w["y"]) for w in waypoints if w.get("y") is not None]
        bounds = (min(rx) - args.margin_m, min(ry) - args.margin_m,
                  max(rx) + args.margin_m, max(ry) + args.margin_m)
        print("route bounds + %.0f m margin: %s" % (args.margin_m, str(bounds)))
    else:
        bounds = (xyz[:, 0].min(), xyz[:, 1].min(), xyz[:, 0].max(), xyz[:, 1].max())

    image, ground, width, height = render(
        xyz, args.resolution, bounds, args.ground_percentile, args.wall_m)
    out_png = args.out + ".png"
    image.save(out_png, optimize=True)
    meta = {
        "frame": "map",
        "source_pcd": os.path.basename(args.pcd),
        "resolution_m_per_pixel": args.resolution,
        # Bottom-left corner of the image in map coordinates, ROS map_server
        # convention: pixel (0, height-1) sits here.
        "origin": [bounds[0], bounds[1], 0.0],
        "width": width,
        "height": height,
        "ground_z": ground,
        "wall_threshold_m": args.wall_m,
    }
    with open(args.out + ".json", "w") as handle:
        json.dump(meta, handle, indent=2)
    print("wrote %s (%dx%d, %.1f kB) and %s.json"
          % (out_png, width, height, os.path.getsize(out_png) / 1024.0, args.out))


if __name__ == "__main__":
    main()
