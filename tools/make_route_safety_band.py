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
import math
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import cKDTree

BIN = 0.3
MAX_LAT = 6.0
STEP = 0.07
# Matches safety_band.DROP_SEVERE_M: a fall the wheel drops off rather than a
# lip it rides over. Kept here so the recorded edge kind and the consumer's
# clearance policy cannot drift apart.
DROP_SEVERE_M = 0.12
STATION_SPACING = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
# Sub-bin refinement of the edge found by the coarse walk. BIN has to stay
# wide enough that its 15th percentile is a stable ground estimate, but the
# edge itself can then be located far more precisely inside it.
FINE = 0.05
MIN_SLICE_POINTS = 3
# Ground search window, relative to the route's own height at that station.
# Taking the 15th percentile of a whole vertical column only finds ground
# where ground points dominate it. On a map merged from several passes the
# walls, canopies and foliage above the pavement outnumber it, and the
# percentile lands on a building face metres up: measured on
# merged_0707_0725, station 52 reported "ground" at +7.3 m and 105 of 358
# stations collapsed to zero usable width. The route height is known per
# station, so ground is searched only in a band around it.
GROUND_BELOW = 2.0
GROUND_ABOVE = 0.3

pcd_path, route_path, out_prefix = sys.argv[1], sys.argv[2], sys.argv[3]

with open(pcd_path, "rb") as f:
    header = b""
    while not header.endswith(b"DATA binary\n"):
        header += f.read(1)
    cloud = np.frombuffer(f.read(), dtype=np.float32).reshape(-1, 4)[:, :3]
cloud = cloud[np.isfinite(cloud).all(axis=1)]
cloud_xy_tree = cKDTree(cloud[:, :2])

route = json.load(open(route_path))
wp = np.array([[w["x"], w["y"]] for w in route["waypoints"]])
if not all("z" in w for w in route["waypoints"]):
    sys.exit("route waypoints need a z field to locate ground; re-export the "
             "route with height (see routes/README or the extraction step)")
wz = np.array([w["z"] for w in route["waypoints"]], dtype=float)

# densify route to 1 m stations, carrying height along
stations = [wp[0]]
station_z = [wz[0]]
for i in range(1, len(wp)):
    p, z = wp[i], wz[i]
    while np.linalg.norm(p - stations[-1]) >= STATION_SPACING:
        d = p - stations[-1]
        remaining = np.linalg.norm(d)
        stations.append(stations[-1] + d / remaining * STATION_SPACING)
        # linear height interpolation over the remaining span to waypoint i
        station_z.append(station_z[-1] +
                         (z - station_z[-1]) * (STATION_SPACING / remaining))
stations = np.array(stations)
station_z = np.array(station_z)

bands = []
n_bins = int(2 * MAX_LAT / BIN)
for k in range(len(stations)):
    center = stations[k].copy()
    nxt = stations[min(k + 1, len(stations) - 1)]
    prv = stations[max(k - 1, 0)]
    d = np.subtract(nxt, prv, dtype=float)
    norm = np.linalg.norm(d)
    if norm < 1e-6:
        continue
    d = np.divide(d, norm)
    normal = np.array([-d[1], d[0]])
    nearby = cloud_xy_tree.query_ball_point(
        center, math.hypot(MAX_LAT, 0.5))
    local_cloud = cloud[np.asarray(nearby, dtype=np.int64)]
    rel = np.subtract(local_cloud[:, :2], center)
    along = np.einsum("ij,j->i", rel, d)
    lat = np.einsum("ij,j->i", rel, normal)
    here_z = station_z[k]
    m = (np.abs(along) < 0.5) & (np.abs(lat) < MAX_LAT) & \
        (local_cloud[:, 2] > here_z - GROUND_BELOW) & \
        (local_cloud[:, 2] < here_z + GROUND_ABOVE)
    ls, zs2 = lat[m], local_cloud[m, 2]
    bins = np.floor((ls + MAX_LAT) / BIN).astype(int)
    prof = {}
    for b in range(n_bins):
        sel = zs2[bins == b]
        if len(sel) >= 3:
            prof[b] = float(np.percentile(sel, 15))
    mid = n_bins // 2
    ref_bins = [prof[b] for b in range(mid - 2, mid + 2) if b in prof]
    if not ref_bins:
        # No ground reference at this station: nothing was measured, so
        # both edges are unknown rather than open.
        bands.append((center[0], center[1], float(np.degrees(
            np.arctan2(d[1], d[0]))), 0.0, 0.0, -1.0, -1.0,
            "unscanned", "unscanned", 0.0, 0.0))
        continue

    def walk(direction):
        """Outward limit plus HOW FAR DOWN the terrain goes past it.

        The limit alone cannot distinguish "the pavement ends at a 15 cm
        kerb into a road" from "the surface tilts gently" or "the scan
        simply has no returns here". The follower needs that distinction
        to decide how much clearance to insist on, so the depth of the
        step that stopped the walk is measured and reported alongside.
        """
        limit = 0.0
        prev = float(np.median(ref_bins))
        reference = prev
        stopped_at = None
        for i in range(1, mid):
            b = mid + direction * i - (1 if direction < 0 else 0)
            if b not in prof:
                stopped_at = None  # ran out of returns, not a seen step
                break
            if abs(prof[b] - prev) > STEP:
                stopped_at = b
                break
            prev = prof[b]
            limit = i * BIN
        if stopped_at is not None:
            # The coarse walk only knows WHICH BIN the step is in, so the
            # edge is quantised to BIN. That quantisation is not a rounding
            # detail: it lands the kerb anywhere within +-BIN/2, and the
            # follower has to hold that much extra clearance to cover it -
            # on this route +-0.15 m, the same size as the margins being
            # traded. Re-walk the stopping bin in FINE slices to place the
            # edge where the height actually breaks.
            outward = direction * ls
            refined = limit
            level = prev
            edge = limit
            while edge < limit + BIN - 1e-9:
                near, far = edge, edge + FINE
                slice_z = zs2[(outward >= near) & (outward < far)]
                if len(slice_z) < MIN_SLICE_POINTS:
                    break
                z = float(np.percentile(slice_z, 15))
                if abs(z - level) > STEP:
                    break
                level = z
                refined = far
                edge = far
            limit = refined
        if stopped_at is None:
            # No observed step: either open ground out to MAX_LAT, or the
            # scan has no returns past the limit. A gap is NOT evidence of
            # safety, so report it as unknown (-1) and let the consumer
            # apply its own policy.
            if limit >= (mid - 1) * BIN:
                return limit, 0.0, "open", 0.0
            return limit, -1.0, "unscanned", 0.0
        # How far the ground keeps falling over the next ~1 m past the step -
        # and how far it RISES, which is the half this used to throw away.
        # depth is max(0, ...), so a kerb, wall or planter that steps UP
        # reported exactly 0.0 and was read downstream as "nothing to fall
        # off": EDGE_MARGIN instead of the full chair half width, and
        # invisible to hazard_clearance. Driving into a raised kerb is not
        # safer than driving off a dropped one, so the kind is recorded and
        # the consumer decides.
        depth = 0.0
        rise = 0.0
        for j in range(0, 4):
            b = stopped_at + direction * j
            if b in prof:
                depth = max(depth, reference - prof[b])
                rise = max(rise, prof[b] - reference)
        if depth >= DROP_SEVERE_M:
            kind = "drop"
        elif depth > 0.0:
            kind = "lip"
        else:
            kind = "step_up"
        return limit, round(float(depth), 3), kind, round(float(rise), 3)

    left, left_drop, left_kind, left_rise = walk(+1)
    right, right_drop, right_kind, right_rise = walk(-1)
    bands.append((float(center[0]), float(center[1]),
                  float(np.degrees(np.arctan2(d[1], d[0]))),
                  round(left, 2), round(right, 2),
                  left_drop, right_drop,
                  left_kind, right_kind, left_rise, right_rise))

lefts = np.array([b[3] for b in bands])
rights = np.array([b[4] for b in bands])
# 3-station median smoothing against single-station noise
def smooth(a):
    out = a.copy()
    for i in range(1, len(a) - 1):
        out[i] = np.median(a[i - 1:i + 2])
    return out
lefts, rights = smooth(lefts), smooth(rights)
# drops are NOT smoothed: a single station seeing a real kerb is exactly
# the signal that must survive, and a median filter would erase it.
bands = [(b[0], b[1], b[2], float(l), float(r), b[5], b[6],
          b[7], b[8], b[9], b[10])
         for b, l, r in zip(bands, lefts, rights)]

width = lefts + rights
drops = np.array([max(b[5], b[6]) for b in bands])
print("stations: %d" % len(bands))
print("band width: min %.1f m, median %.1f m" % (width.min(), np.median(width)))
narrow = [i for i, w in enumerate(width) if w < 0.9]
print("stations narrower than 0.9 m: %s" % (narrow if narrow else "none"))
from collections import Counter
kinds = Counter([b[7] for b in bands]) + Counter([b[8] for b in bands])
print("edge kind: " + ", ".join("%s=%d" % kv for kv in sorted(kinds.items())))
rises = np.array([max(b[9], b[10]) for b in bands])
seen_rise = rises[rises > 0]
if len(seen_rise):
    print("measured step RISE: median %.2f m, p90 %.2f m, max %.2f m"
          % (np.median(seen_rise), np.percentile(seen_rise, 90),
             seen_rise.max()))
seen = drops[drops > 0]
if len(seen):
    print("measured step depth: median %.2f m, p90 %.2f m, max %.2f m"
          % (np.median(seen), np.percentile(seen, 90), seen.max()))

with open(out_prefix + ".json", "w") as f:
    json.dump({"frame": "map", "station_spacing_m": STATION_SPACING,
               "stations": [{"x": b[0], "y": b[1], "heading_deg": b[2],
                             "left_m": b[3], "right_m": b[4],
                             "left_drop_m": b[5], "right_drop_m": b[6],
                             "left_kind": b[7], "right_kind": b[8],
                             "left_rise_m": b[9], "right_rise_m": b[10]}
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
for x, y, hdg, l, r, _ldrop, _rdrop, _lk, _rk, _lr, _rr in bands:
    h = np.radians(hdg)
    n = np.array([-np.sin(h), np.cos(h)])
    left_pts.append(px(x + n[0] * l, y + n[1] * l))
    right_pts.append(px(x - n[0] * r, y - n[1] * r))
draw.line([px(b[0], b[1]) for b in bands], fill=(60, 130, 255), width=2)
draw.line(left_pts, fill=(70, 220, 90), width=1)
draw.line(right_pts, fill=(70, 220, 90), width=1)
pil.save(out_prefix + "_preview.png")
print("saved %s.json / %s_preview.png" % (out_prefix, out_prefix))
