#!/usr/bin/env python3
"""Generate a sidewalk/road traversability mask from a localization map.

Ground cells are segmented into connected regions that are cut wherever a
curb-like height step (6-35 cm) separates neighbours. Regions touched by the
recorded driving route (which was driven on the sidewalk) are labeled
sidewalk; every other ground region (road, planting strips) is blocked.
Gentle ramps stay connected, so crossings and curb cuts open naturally.

Outputs <prefix>.pgm / <prefix>.yaml (ROS map_server format, sidewalk =
free) and <prefix>_preview.png (green sidewalk, red other ground, black
obstacles, yellow curb edges, blue route).
"""

import argparse
import math

import numpy as np


def load_pcd_xyz(path):
    with open(path, "rb") as f:
        header = b""
        while not header.endswith(b"DATA binary\n"):
            byte = f.read(1)
            if not byte or len(header) > 8192:
                raise RuntimeError("unsupported PCD (need binary XYZI)")
            header += byte
        fields = 4 if b"intensity" in header else 3
        data = np.frombuffer(f.read(), dtype=np.float32)
    points = data.reshape(-1, fields)[:, :3]
    return points[np.isfinite(points).all(axis=1)]


def load_route(bag_path):
    import rosbag

    track = []
    with rosbag.Bag(bag_path) as bag:
        for _, message, _ in bag.read_messages(topics=["/fast_lio_icp/pose"]):
            position = message.pose.pose.position
            track.append((position.x, position.y))
    if not track:
        raise RuntimeError("no /fast_lio_icp/pose in route bag")
    return np.array(track)


class UnionFind:
    def __init__(self, size):
        self.parent = np.arange(size, dtype=np.int64)

    def find(self, a):
        root = a
        while self.parent[root] != root:
            root = self.parent[root]
        while self.parent[a] != root:
            self.parent[a], a = root, self.parent[a]
        return root

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[rb] = ra


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--route-bag", required=True)
    parser.add_argument("--out", required=True, help="output path prefix")
    parser.add_argument("--cell", type=float, default=0.5)
    parser.add_argument("--curb-min", type=float, default=0.08)
    parser.add_argument("--curb-max", type=float, default=0.35)
    parser.add_argument("--connect-max", type=float, default=0.07)
    parser.add_argument("--min-cell-points", type=int, default=4)
    parser.add_argument("--route-seed-radius", type=float, default=0.8)
    args = parser.parse_args()

    points = load_pcd_xyz(args.pcd)
    route = load_route(args.route_bag)
    print("map points: %d, route poses: %d" % (len(points), len(route)))

    cell = args.cell
    min_x, min_y = points[:, 0].min(), points[:, 1].min()
    ci = np.floor((points[:, 0] - min_x) / cell).astype(np.int64)
    cj = np.floor((points[:, 1] - min_y) / cell).astype(np.int64)
    width, height = ci.max() + 1, cj.max() + 1
    print("grid: %d x %d cells (%.1f x %.1f m)"
          % (width, height, width * cell, height * cell))

    flat = ci * height + cj
    order = np.argsort(flat, kind="stable")
    flat_sorted = flat[order]
    z_sorted = points[order, 2]
    unique_flat, starts, counts = np.unique(
        flat_sorted, return_index=True, return_counts=True
    )

    ground = np.full(width * height, np.nan, dtype=np.float32)
    obstacle = np.zeros(width * height, dtype=bool)
    for uf, start, count in zip(unique_flat, starts, counts):
        if count < args.min_cell_points:
            continue
        zs = z_sorted[start:start + count]
        g = np.percentile(zs, 15)
        ground[uf] = g
        if np.count_nonzero((zs > g + 0.35) & (zs < g + 2.0)) >= 3:
            obstacle[uf] = True

    ground = ground.reshape(width, height)
    obstacle = obstacle.reshape(width, height)

    padded = np.pad(ground, 1, constant_values=np.nan)
    stack = np.stack([padded[1 + dx:1 + dx + width, 1 + dy:1 + dy + height]
                      for dx in (-1, 0, 1) for dy in (-1, 0, 1)])
    with np.errstate(all="ignore"):
        smoothed = np.nanmedian(stack, axis=0)
    keep = np.isfinite(ground)
    ground = np.where(keep, smoothed, np.nan).astype(np.float32)
    valid = np.isfinite(ground)
    print("ground cells: %d, obstacle cells: %d"
          % (valid.sum(), obstacle.sum()))

    walkable = valid & ~obstacle
    uf = UnionFind(width * height)
    curb_edge = np.zeros((width, height), dtype=bool)
    for dx, dy in ((1, 0), (0, 1), (1, 1)):
        a = walkable[: width - dx if dx else width, : height - dy if dy else height]
        b = walkable[dx:, dy:]
        dz = np.abs(ground[: width - dx if dx else width,
                           : height - dy if dy else height] - ground[dx:, dy:])
        both = a & b
        connect = both & (dz <= args.connect_max)
        curb = (both & (dz >= args.curb_min) & (dz <= args.curb_max)
                if (dx, dy) != (1, 1) else np.zeros_like(both))
        idx_a = np.flatnonzero(np.pad(connect, ((0, dx), (0, dy))).ravel())
        for ia in idx_a:
            uf.union(ia, ia + dx * height + dy)
        curb_edge[: width - dx if dx else width,
                  : height - dy if dy else height] |= curb

    roots = np.array([uf.find(i) if walkable.ravel()[i] else -1
                      for i in range(width * height)])

    seed_cells = set()
    steps = max(1, int(args.route_seed_radius / cell))
    for x, y in route:
        base_i = int((x - min_x) / cell)
        base_j = int((y - min_y) / cell)
        for di in range(-steps, steps + 1):
            for dj in range(-steps, steps + 1):
                i, j = base_i + di, base_j + dj
                if 0 <= i < width and 0 <= j < height and walkable[i, j]:
                    seed_cells.add(i * height + j)
    sidewalk_roots = {roots[s] for s in seed_cells if roots[s] >= 0}
    sidewalk = np.isin(roots, list(sidewalk_roots)).reshape(width, height)
    sidewalk &= walkable
    other_ground = walkable & ~sidewalk
    print("sidewalk cells: %d (%.1f%% of walkable), other ground: %d"
          % (sidewalk.sum(), 100.0 * sidewalk.sum() / max(1, walkable.sum()),
             other_ground.sum()))

    from PIL import Image

    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[...] = 90
    rgb[other_ground.T] = (200, 60, 60)
    rgb[sidewalk.T] = (60, 180, 75)
    rgb[obstacle.T] = (0, 0, 0)
    rgb[curb_edge.T] = (250, 210, 40)
    for x, y in route[::5]:
        i, j = int((x - min_x) / cell), int((y - min_y) / cell)
        if 0 <= i < width and 0 <= j < height:
            rgb[j, i] = (40, 90, 250)
    Image.fromarray(rgb[::-1]).save(args.out + "_preview.png")

    occupancy = np.full((width, height), 205, dtype=np.uint8)
    occupancy[sidewalk] = 254
    occupancy[other_ground] = 0
    occupancy[obstacle] = 0
    Image.fromarray(occupancy.T[::-1]).save(args.out + ".pgm")
    with open(args.out + ".yaml", "w") as f:
        f.write(
            "image: %s.pgm\nresolution: %.3f\norigin: [%.3f, %.3f, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
            % (args.out.split("/")[-1], cell, min_x, min_y)
        )
    print("wrote %s_preview.png, %s.pgm, %s.yaml" % ((args.out,) * 3))


if __name__ == "__main__":
    main()
