#!/usr/bin/env python3
"""Build a drop-hazard mask from the localization map.

Detects curb-like height discontinuities (7-40 cm steps between adjacent
ground cells) in the pre-scanned map and dilates them by a safety margin.
The follower checks its map-frame corridor against this mask, so near-field
drops are handled from the map instead of the MID360's upward-only FOV.

Usage: make_drop_hazard_mask.py <map.pcd> <route.json> <out-prefix>
Outputs <out-prefix>.npz (mask, origin, cell) and <out-prefix>_preview.png.
"""

import json
import sys

import numpy as np
from PIL import Image, ImageDraw

CELL = 0.3
STEP_MIN = 0.07
STEP_MAX = 0.45
MIN_PTS = 3
DILATE_CELLS = 1

pcd_path, route_path, out_prefix = sys.argv[1], sys.argv[2], sys.argv[3]

with open(pcd_path, "rb") as f:
    header = b""
    while not header.endswith(b"DATA binary\n"):
        header += f.read(1)
    cloud = np.frombuffer(f.read(), dtype=np.float32).reshape(-1, 4)[:, :3]
cloud = cloud[np.isfinite(cloud).all(axis=1)]

min_x, min_y = float(cloud[:, 0].min()), float(cloud[:, 1].min())
ci = np.floor((cloud[:, 0] - min_x) / CELL).astype(np.int64)
cj = np.floor((cloud[:, 1] - min_y) / CELL).astype(np.int64)
W, H = int(ci.max()) + 1, int(cj.max()) + 1

flat = ci * H + cj
order = np.argsort(flat)
fs, zs = flat[order], cloud[order, 2]
uf, starts, counts = np.unique(fs, return_index=True, return_counts=True)
ground = np.full(W * H, np.nan, np.float32)
for u, s, c in zip(uf, starts, counts):
    if c >= MIN_PTS:
        ground[u] = np.percentile(zs[s:s + c], 15)
ground = ground.reshape(W, H)
print("grid %dx%d, ground cells %d" % (W, H, np.isfinite(ground).sum()))

hazard = np.zeros((W, H), bool)
for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)):
    a = ground[max(0, -dx):W - max(0, dx), max(0, -dy):H - max(0, dy)]
    b = ground[max(0, dx):W - max(0, -dx), max(0, dy):H - max(0, -dy)]
    dz = np.abs(a - b)
    edge = np.isfinite(dz) & (dz >= STEP_MIN) & (dz <= STEP_MAX)
    hazard[max(0, -dx):W - max(0, dx), max(0, -dy):H - max(0, dy)] |= edge
    hazard[max(0, dx):W - max(0, -dx), max(0, dy):H - max(0, -dy)] |= edge
print("raw hazard cells: %d" % hazard.sum())

for _ in range(DILATE_CELLS):
    d = hazard.copy()
    d[1:, :] |= hazard[:-1, :]
    d[:-1, :] |= hazard[1:, :]
    d[:, 1:] |= hazard[:, :-1]
    d[:, :-1] |= hazard[:, 1:]
    hazard = d
print("dilated hazard cells: %d" % hazard.sum())

np.savez_compressed(out_prefix + ".npz", hazard=hazard,
                    origin=np.array([min_x, min_y]), cell=CELL)

route = json.load(open(route_path))
wp = np.array([[w["x"], w["y"]] for w in route["waypoints"]])

img = np.zeros((H, W, 3), np.uint8)
img[...] = 35
img[np.isfinite(ground).T] = (90, 90, 90)
img[hazard.T] = (215, 60, 60)
pil = Image.fromarray(img[::-1])
draw = ImageDraw.Draw(pil)

def px(p):
    return (int((p[0] - min_x) / CELL), H - 1 - int((p[1] - min_y) / CELL))

draw.line([px(p) for p in wp], fill=(60, 130, 255), width=2)
for p in wp:
    x, y = px(p)
    draw.ellipse([x - 2, y - 2, x + 2, y + 2], fill=(255, 200, 60))
pil.save(out_prefix + "_preview.png")

# route clearance report
ox, oy = min_x, min_y
danger = []
for index, p in enumerate(wp):
    i0, j0 = int((p[0] - ox) / CELL), int((p[1] - oy) / CELL)
    radius = int(0.6 / CELL) + 1
    hit = False
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            i, j = i0 + di, j0 + dj
            if 0 <= i < W and 0 <= j < H and hazard[i, j] and \
                    (di * di + dj * dj) * CELL * CELL <= 0.36:
                hit = True
    if hit:
        danger.append(index)
print("waypoints with hazard within 0.6 m: %s"
      % (danger if danger else "none"))
print("saved %s.npz / %s_preview.png" % (out_prefix, out_prefix))
