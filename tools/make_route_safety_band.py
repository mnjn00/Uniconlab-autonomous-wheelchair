#!/usr/bin/env python3
"""Compute the drop-free lateral band along the route from the map.

For every 1 m route station, a 1 m-long lateral strip of map points is
binned (0.3 m) and walked outward from the driven line; the band ends at
the first curb-like step or data gap. The follower must keep the
wheelchair inside this band, so map-known drops (curbs, road edges) are
avoided without needing downward LiDAR view.

Ground extraction is anchored to the DRIVEN HEIGHT, not to a percentile
of whatever the strip contains. A plain low percentile of a lateral bin
silently returns wall/railing/vegetation height wherever structure
dominates the bin, which made the very first outward bin look like a
huge step and collapsed the band to zero width. On the 3-pass merged
map that happened at 11% of stations, so the failure is not rare: the
denser the map, the more non-ground points per bin, the worse a bare
percentile behaves. Every route waypoint carries the z the wheelchair
actually drove at, so the ground near a station is known to within the
sensor mount height; only points inside a window around that height are
eligible to represent ground.

Usage: make_route_safety_band.py <map.pcd|map.ply> <route.json> <out-prefix>
Outputs <out-prefix>.json (stations with left/right limits) and
<out-prefix>_preview.png.
"""

import json
import sys

import numpy as np
from PIL import Image, ImageDraw

BIN = 0.3
MAX_LAT = 6.0
STEP = 0.07
STATION_SPACING = 1.0
# Ground candidates sit below the sensor by roughly the mount height.
# Measured on the 2026-07-27 drive against the merged map: pose z minus
# local ground z was 1.28 m median (p10 0.85, p90 1.54), so a window of
# [drive_z - 2.2, drive_z - 0.3] covers the mount height everywhere while
# still excluding railings, parked cars and walls.
GROUND_BELOW_MIN = 0.3
GROUND_BELOW_MAX = 2.2
# A bin's ground estimate is only trusted with enough eligible points;
# too few means the map does not actually see the ground there, which is
# a data gap and must end the band rather than extend it on noise.
MIN_BIN_POINTS = 3


def load_cloud(path):
    """Read xyz from a binary PCD or a binary_little_endian PLY."""
    with open(path, "rb") as f:
        if path.lower().endswith(".ply"):
            header = b""
            while not header.endswith(b"end_header\n"):
                header += f.read(1)
            text = header.decode(errors="replace")
            n_props = sum(1 for line in text.splitlines()
                          if line.startswith("property "))
            if "binary_little_endian" not in text:
                raise SystemExit("only binary_little_endian PLY is supported")
            data = np.frombuffer(f.read(), dtype=np.float32)
            return data.reshape(-1, n_props)[:, :3]
        header = b""
        while not header.endswith(b"DATA binary\n"):
            header += f.read(1)
        fields = [line for line in header.decode(errors="replace").splitlines()
                  if line.startswith("FIELDS")]
        n_props = len(fields[0].split()) - 1 if fields else 4
        data = np.frombuffer(f.read(), dtype=np.float32)
        return data.reshape(-1, n_props)[:, :3]


def main():
    pcd_path, route_path, out_prefix = sys.argv[1], sys.argv[2], sys.argv[3]

    cloud = load_cloud(pcd_path)
    cloud = cloud[np.isfinite(cloud).all(axis=1)]

    route = json.load(open(route_path))
    waypoints = route["waypoints"]
    if not all("z" in w for w in waypoints):
        raise SystemExit(
            "route waypoints must carry z (the height the chair drove at) - "
            "ground extraction is anchored to it")
    wp = np.array([[w["x"], w["y"]] for w in waypoints])
    wp_z = np.array([w["z"] for w in waypoints])

    # densify route to 1 m stations, carrying drive height along
    stations = [wp[0]]
    station_z = [wp_z[0]]
    for k in range(1, len(wp)):
        p, pz = wp[k], wp_z[k]
        while np.linalg.norm(p - stations[-1]) >= STATION_SPACING:
            d = p - stations[-1]
            n = np.linalg.norm(d)
            frac = STATION_SPACING / n
            stations.append(stations[-1] + d * frac)
            station_z.append(station_z[-1] + (pz - station_z[-1]) * frac)
    stations = np.array(stations)
    station_z = np.array(station_z)

    bands = []
    n_bins = int(2 * MAX_LAT / BIN)
    mid = n_bins // 2
    for k in range(len(stations)):
        center = stations[k]
        drive_z = station_z[k]
        nxt = stations[min(k + 1, len(stations) - 1)]
        prv = stations[max(k - 1, 0)]
        d = nxt - prv
        norm = np.linalg.norm(d)
        if norm < 1e-6:
            continue
        d = d / norm
        normal = np.array([-d[1], d[0]])
        rel = cloud[:, :2] - center
        along = rel @ d
        lat = rel @ normal
        # ground window anchored on the height the wheelchair drove at
        in_strip = (np.abs(along) < 0.5) & (np.abs(lat) < MAX_LAT) & \
            (cloud[:, 2] > drive_z - GROUND_BELOW_MAX) & \
            (cloud[:, 2] < drive_z - GROUND_BELOW_MIN)
        ls, zs2 = lat[in_strip], cloud[in_strip, 2]
        bins = np.floor((ls + MAX_LAT) / BIN).astype(int)
        prof = {}
        for b in range(n_bins):
            sel = zs2[bins == b]
            if len(sel) >= MIN_BIN_POINTS:
                prof[b] = float(np.percentile(sel, 15))
        ref_bins = [prof[b] for b in range(mid - 2, mid + 2) if b in prof]
        if not ref_bins:
            bands.append((float(center[0]), float(center[1]), float(np.degrees(
                np.arctan2(d[1], d[0]))), 0.0, 0.0))
            continue

        def walk(direction):
            limit = 0.0
            prev = float(np.median(ref_bins))
            for i in range(1, mid):
                b = mid + direction * i - (1 if direction < 0 else 0)
                if b not in prof:
                    break
                if abs(prof[b] - prev) > STEP:
                    break
                prev = prof[b]
                limit = i * BIN
            return limit

        left = walk(+1)
        right = walk(-1)
        bands.append((float(center[0]), float(center[1]),
                      float(np.degrees(np.arctan2(d[1], d[0]))),
                      round(left, 2), round(right, 2)))

    lefts = np.array([b[3] for b in bands])
    rights = np.array([b[4] for b in bands])

    # 3-station median smoothing against single-station noise
    def smooth(a):
        out = a.copy()
        for i in range(1, len(a) - 1):
            out[i] = np.median(a[i - 1:i + 2])
        return out

    lefts, rights = smooth(lefts), smooth(rights)
    bands = [(b[0], b[1], b[2], float(l), float(r))
             for b, l, r in zip(bands, lefts, rights)]

    width = lefts + rights
    print("stations: %d" % len(bands))
    print("band width: min %.1f m, median %.1f m" % (width.min(),
                                                     np.median(width)))
    zero = int(np.sum(width == 0.0))
    print("stations with NO band at all: %d (%.0f%%)"
          % (zero, 100.0 * zero / len(bands)))
    narrow = [i for i, w in enumerate(width) if w < 0.9]
    print("stations narrower than 0.9 m: %s" % (narrow if narrow else "none"))

    with open(out_prefix + ".json", "w") as f:
        json.dump({"frame": "map", "station_spacing_m": STATION_SPACING,
                   "stations": [{"x": b[0], "y": b[1], "heading_deg": b[2],
                                 "left_m": b[3], "right_m": b[4]}
                                for b in bands]}, f, indent=1)

    CELL = 0.4
    min_x, min_y = cloud[:, 0].min(), cloud[:, 1].min()
    W = int((cloud[:, 0].max() - min_x) / CELL) + 1
    H = int((cloud[:, 1].max() - min_y) / CELL) + 1
    img = np.zeros((H, W, 3), np.uint8)
    img[...] = 35
    ci = ((cloud[:, 0] - min_x) / CELL).astype(int)
    cj = ((cloud[:, 1] - min_y) / CELL).astype(int)
    img[cj, ci] = (90, 90, 90)
    pil = Image.fromarray(img[::-1])
    draw = ImageDraw.Draw(pil)

    def px(x, y):
        return (int((x - min_x) / CELL), H - 1 - int((y - min_y) / CELL))

    left_pts, right_pts = [], []
    for x, y, hdg, l, r in bands:
        h = np.radians(hdg)
        n = np.array([-np.sin(h), np.cos(h)])
        left_pts.append(px(x + n[0] * l, y + n[1] * l))
        right_pts.append(px(x - n[0] * r, y - n[1] * r))
    draw.line([px(b[0], b[1]) for b in bands], fill=(60, 130, 255), width=2)
    draw.line(left_pts, fill=(70, 220, 90), width=1)
    draw.line(right_pts, fill=(70, 220, 90), width=1)
    pil.save(out_prefix + "_preview.png")
    print("saved %s.json / %s_preview.png" % (out_prefix, out_prefix))


if __name__ == "__main__":
    main()
