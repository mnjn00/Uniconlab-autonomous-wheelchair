#!/usr/bin/env python3
"""Compute the drop-free lateral band along the route from the map.

For every 1 m route station, a 1 m-long lateral strip of map points is
binned (0.3 m) and walked outward from the driven line; the band ends at
the first curb-like step (>7 cm bin-to-bin) or data gap. The follower must
keep the wheelchair inside this band, so map-known drops (curbs, road
edges) are avoided without needing downward LiDAR view.

Usage: make_route_safety_band.py <map.pcd> <route.json> <out-prefix>
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

pcd_path, route_path, out_prefix = sys.argv[1], sys.argv[2], sys.argv[3]

with open(pcd_path, "rb") as f:
    header = b""
    while not header.endswith(b"DATA binary\n"):
        header += f.read(1)
    cloud = np.frombuffer(f.read(), dtype=np.float32).reshape(-1, 4)[:, :3]
cloud = cloud[np.isfinite(cloud).all(axis=1)]

route = json.load(open(route_path))
wp = np.array([[w["x"], w["y"]] for w in route["waypoints"]])

# densify route to 1 m stations
stations = [wp[0]]
for p in wp[1:]:
    while np.linalg.norm(p - stations[-1]) >= STATION_SPACING:
        d = p - stations[-1]
        stations.append(stations[-1] + d / np.linalg.norm(d) * STATION_SPACING)
stations = np.array(stations)

bands = []
n_bins = int(2 * MAX_LAT / BIN)
for k in range(len(stations)):
    center = stations[k]
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
    m = (np.abs(along) < 0.5) & (np.abs(lat) < MAX_LAT)
    ls, zs2 = lat[m], cloud[m, 2]
    bins = np.floor((ls + MAX_LAT) / BIN).astype(int)
    prof = {}
    for b in range(n_bins):
        sel = zs2[bins == b]
        if len(sel) >= 3:
            prof[b] = float(np.percentile(sel, 15))
    mid = n_bins // 2
    ref_bins = [prof[b] for b in range(mid - 2, mid + 2) if b in prof]
    if not ref_bins:
        bands.append((center[0], center[1], float(np.degrees(
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
print("band width: min %.1f m, median %.1f m" % (width.min(), np.median(width)))
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
