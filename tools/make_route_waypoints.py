"""Curvature-adaptive waypoints from the 2026-07-07 mapping drive trajectory.

Straight stretches get sparse waypoints (up to 6 m apart), curves get dense
ones (down to 1.5 m). Uses the outbound start->destination leg only.
"""

import json
import sys

import numpy as np
from PIL import Image, ImageDraw

traj_path, pcd_path, out_prefix = sys.argv[1], sys.argv[2], sys.argv[3]

rows = np.loadtxt(traj_path)
xy = rows[:, 1:3]

# 0.2 m polyline
poly = [xy[0]]
for p in xy:
    if np.linalg.norm(p - poly[-1]) >= 0.2:
        poly.append(p)
poly = np.array(poly)

# outbound leg: start -> farthest point
d_from_start = np.linalg.norm(poly - poly[0], axis=1)
turn = int(np.argmax(d_from_start))
leg = poly[: turn + 1]
seg = np.linalg.norm(np.diff(leg, axis=0), axis=1)
length = seg.sum()
print("full traj %.0f m, outbound leg %.0f m (turnaround at %.0f%%)"
      % (np.linalg.norm(np.diff(poly, axis=0), axis=1).sum(), length,
         100.0 * turn / len(poly)))

# smooth (about 1 m window) and per-point curvature (rad/m over +-1 m)
kernel = 5
pad = np.vstack([leg[:1].repeat(kernel, 0), leg, leg[-1:].repeat(kernel, 0)])
smooth = np.vstack([
    pad[i:len(pad) - 2 * kernel + i] for i in range(2 * kernel + 1)
]).reshape(2 * kernel + 1, -1, 2).mean(axis=0)

headings = np.arctan2(*np.diff(smooth, axis=0).T[::-1])
window = 5
curvature = np.zeros(len(smooth))
for i in range(len(smooth)):
    a = headings[max(0, i - window)]
    b = headings[min(len(headings) - 1, i + window - 1)]
    dh = np.arctan2(np.sin(b - a), np.cos(b - a))
    arc = 0.2 * (min(len(headings) - 1, i + window - 1) - max(0, i - window) + 1)
    curvature[i] = abs(dh) / max(arc, 0.2)

def spacing(k):
    return float(np.clip(6.0 / (1.0 + 12.0 * k), 1.5, 6.0))

waypoints = [0]
acc = 0.0
for i in range(1, len(smooth)):
    acc += np.linalg.norm(smooth[i] - smooth[i - 1])
    if acc >= spacing(curvature[i]):
        waypoints.append(i)
        acc = 0.0
if len(smooth) - 1 - waypoints[-1] > 3:
    waypoints.append(len(smooth) - 1)

wp = []
for i in waypoints:
    yaw = float(headings[min(i, len(headings) - 1)])
    wp.append({"x": round(float(smooth[i][0]), 2),
               "y": round(float(smooth[i][1]), 2),
               "yaw_deg": round(float(np.degrees(yaw)), 1)})
print("waypoints: %d (start (%.1f,%.1f) -> end (%.1f,%.1f))"
      % (len(wp), wp[0]["x"], wp[0]["y"], wp[-1]["x"], wp[-1]["y"]))
straight = sum(1 for i in waypoints if curvature[i] < 0.03)
print("spacing: straight-ish %d, curved %d" % (straight, len(wp) - straight))

with open(out_prefix + ".json", "w") as f:
    json.dump({"frame": "map", "source": "traj_lidar.txt 2026-07-07 outbound leg",
               "count": len(wp), "waypoints": wp}, f, indent=1)

# render over map ground
with open(pcd_path, "rb") as f:
    header = b""
    while not header.endswith(b"DATA binary\n"):
        header += f.read(1)
    cloud = np.frombuffer(f.read(), dtype=np.float32).reshape(-1, 4)[:, :3]
cloud = cloud[np.isfinite(cloud).all(axis=1)]
CELL = 0.4
min_x, min_y = cloud[:, 0].min(), cloud[:, 1].min()
W = int((cloud[:, 0].max() - min_x) / CELL) + 1
H = int((cloud[:, 1].max() - min_y) / CELL) + 1
img = np.zeros((H, W, 3), np.uint8)
img[...] = 30
ci = ((cloud[:, 0] - min_x) / CELL).astype(int)
cj = ((cloud[:, 1] - min_y) / CELL).astype(int)
img[cj, ci] = (95, 95, 95)

def px(p):
    return (int((p[0] - min_x) / CELL), H - 1 - int((p[1] - min_y) / CELL))

pil = Image.fromarray(img[::-1])
draw = ImageDraw.Draw(pil)
draw.line([px(p) for p in smooth[::3]], fill=(60, 120, 255), width=1)
for index, i in enumerate(waypoints):
    x, y = px(smooth[i])
    r = 3
    color = (255, 170, 40) if curvature[i] < 0.03 else (255, 60, 60)
    draw.ellipse([x - r, y - r, x + r, y + r], fill=color)
    if index % 5 == 0:
        draw.text((x + 4, y - 10), str(index), fill=(240, 240, 240))
sx, sy = px(smooth[0])
ex, ey = px(smooth[-1])
draw.text((sx + 5, sy + 5), "START", fill=(80, 255, 80))
draw.text((ex + 5, ey + 5), "GOAL", fill=(255, 120, 255))

pil.save(out_prefix + ".png")
print("saved %s.json / %s.png" % (out_prefix, out_prefix))
