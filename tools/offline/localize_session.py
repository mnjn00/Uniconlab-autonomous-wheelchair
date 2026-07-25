#!/usr/bin/env python3
"""Localize every scan of a mapping bag against the existing 07/07 map.

Why this instead of running SLAM again: the waypoint route and the safety
band are expressed in the 07/07 map frame, so a fresh SLAM solution would
invalidate all of them. Anchoring each scan to that map keeps the frame
identical AND removes drift, because every scan is registered absolutely
rather than chained onto the previous one.

Output is one pose per scan (TUM-style JSON), written incrementally so a
power cut - which happened twice on 2026-07-25 - costs only the sweeps
since the last checkpoint. Rerunning resumes from the checkpoint.

The map is only available at 0.2 m voxel resolution (the raw GLIM dump is
gone), so it is used purely as a registration target; the fine detail in
the merged product comes from the raw scans themselves.
"""

import argparse
import json
import math
import os
import time

import numpy as np
import open3d as o3d
import rosbag
import sensor_msgs.point_cloud2 as pc2

LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"

TILE_M = 20.0            # map tiling for cheap local crops
LOCAL_RADIUS_M = 35.0    # local map extent handed to ICP
RECROP_MOVE_M = 8.0      # re-crop (and rebuild the KD-tree) only this often
SCAN_VOXEL_M = 0.25      # source downsample for registration only
# Range-limit the source so every kept point has a chance of a match in the
# cropped target; otherwise far returns depress fitness and make the
# quality gates meaningless. 25 m + RECROP_MOVE_M stays inside LOCAL_RADIUS_M.
MAX_SCAN_RANGE_M = 25.0
MAP_NORMAL_RADIUS_M = 0.6
ICP_MAX_CORR_COARSE_M = 1.5
ICP_MAX_CORR_FINE_M = 0.5
ICP_ITERS = 30
MIN_SCAN_POINTS = 800
# The global search must not run on a sparse frame. The driver's first
# message is padding-only (14 valid points of 20064) and early frames are
# partial; 14 points reach fitness 1.000 at an arbitrary pose, which then
# poisons the whole track. Require a properly filled scan instead.
INIT_MIN_POINTS = 8000
# Extrapolating through failures forever walks the pose off the map with no
# way back, so re-run the global search after a sustained loss.
RELOCALIZE_AFTER = 25
# Track quality gates. Registration that fits this badly means the pose is
# not trustworthy and the scan must not be baked into the map.
MIN_FITNESS = 0.35
MAX_RMSE_M = 0.35
MAX_JUMP_M = 1.0         # 0.1 s at 1.5 m/s is 0.15 m; 1 m is already absurd


def quat_to_R(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    t = R.trace()
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
            y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w, y = (R[0, 2] - R[2, 0]) / s, 0.25 * s
            x, z = (R[0, 1] + R[1, 0]) / s, (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w, z = (R[1, 0] - R[0, 1]) / s, 0.25 * s
            x, y = (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s
    return np.array([x, y, z, w])


def yaw_T(yaw, xyz):
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = xyz
    return T


class TiledMap:
    """The reference map, pre-normalled once and sliced on demand.

    Open3D rebuilds the target KD-tree on every registration_icp call, and
    doing that against 1.85 M points for every one of ~11 500 scans would
    dominate the runtime. Cropping to a local tile neighbourhood keeps each
    tree build small, and the crop is reused until the chair has actually
    moved out of it.
    """

    def __init__(self, pcd_path):
        pcd = o3d.io.read_point_cloud(pcd_path)
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=MAP_NORMAL_RADIUS_M, max_nn=30))
        self.points = np.asarray(pcd.points)
        self.normals = np.asarray(pcd.normals)
        keys = np.floor(self.points[:, :2] / TILE_M).astype(np.int32)
        order = np.lexsort((keys[:, 1], keys[:, 0]))
        sorted_keys = keys[order]
        cuts = np.flatnonzero(
            np.any(np.diff(sorted_keys, axis=0) != 0, axis=1)) + 1
        # blocks of `order` line up with `sorted_keys`, so a block's key is
        # read at its first sorted position, not at its map index
        starts = np.concatenate(([0], cuts))
        self.tiles = {tuple(sorted_keys[s]): b
                      for s, b in zip(starts, np.split(order, cuts))}
        self._crop_center = None
        self._crop_pcd = None

    def local(self, center_xy):
        """Local map around center_xy, reusing the previous crop if close."""
        if (self._crop_center is not None and
                np.linalg.norm(np.asarray(center_xy) - self._crop_center)
                < RECROP_MOVE_M):
            return self._crop_pcd
        reach = int(math.ceil(LOCAL_RADIUS_M / TILE_M))
        cx, cy = (int(math.floor(center_xy[0] / TILE_M)),
                  int(math.floor(center_xy[1] / TILE_M)))
        idx = [self.tiles[(cx + dx, cy + dy)]
               for dx in range(-reach, reach + 1)
               for dy in range(-reach, reach + 1)
               if (cx + dx, cy + dy) in self.tiles]
        if not idx:
            return None
        idx = np.concatenate(idx)
        # tiles are square, so trim to the actual radius: a 25 m disc holds
        # several times fewer points than the tile block around it, and the
        # target size is what every registration_icp tree build pays for
        d = self.points[idx, :2] - np.asarray(center_xy)
        idx = idx[(d * d).sum(axis=1) < LOCAL_RADIUS_M ** 2]
        if len(idx) < 2000:
            return None
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points[idx])
        pcd.normals = o3d.utility.Vector3dVector(self.normals[idx])
        self._crop_center = np.asarray(center_xy, dtype=float)
        self._crop_pcd = pcd
        return pcd


def read_imu(bag_path, topic):
    t, gyro = [], []
    with rosbag.Bag(bag_path) as bag:
        for _, m, _ in bag.read_messages(topics=[topic]):
            t.append(m.header.stamp.to_sec())
            gyro.append((m.angular_velocity.x, m.angular_velocity.y,
                         m.angular_velocity.z))
    return np.array(t), np.array(gyro)


_NP_DTYPE = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
             5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def scan_arrays(msg):
    """Points as (xyz, intensity, seconds-since-scan-start).

    Field offsets are read from the message rather than assumed: today's
    bags carry x,y,z,intensity,tag,line,timestamp (absolute ns, float64)
    while the normalised 07/07 bag carries the original CustomMsg layout
    with offset_time (relative ns) plus lidar_id/reflectivity. Hardcoding
    one layout silently produced garbage for the other.
    """
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    fields = {f.name: f for f in msg.fields}

    def grab(name):
        f = fields[name]
        dt = _NP_DTYPE[f.datatype]
        size = np.dtype(dt).itemsize
        return raw[:, f.offset:f.offset + size].copy().view(dt).reshape(-1)

    xyz = np.stack([grab("x"), grab("y"), grab("z")], axis=1).astype(np.float64)
    if "intensity" in fields:
        inten = grab("intensity").astype(np.float32)
    elif "reflectivity" in fields:
        inten = grab("reflectivity").astype(np.float32)
    else:
        inten = np.zeros(len(xyz), dtype=np.float32)

    if "timestamp" in fields:            # absolute nanoseconds
        t = grab("timestamp").astype(np.float64)
        rel = (t - t[0]) * 1e-9 if len(t) else t
    elif "offset_time" in fields:        # nanoseconds from the header stamp
        rel = grab("offset_time").astype(np.float64) * 1e-9
    else:
        rel = np.zeros(len(xyz))

    ok = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 1e-6)
    return xyz[ok], inten[ok], rel[ok]


def deskew(xyz, rel, gyro_t, gyro_w, t0):
    """Undo rotation within the 0.1 s sweep using the gyro.

    Translation inside one scan is under 15 cm at 1.5 m/s and is corrected
    implicitly by the per-scan registration; the rotational component is
    the part that visibly smears walls, so that is what is removed here.
    """
    if len(rel) == 0:
        return xyz
    rel = rel - rel.min()
    span = rel.max() if rel.max() > 1e-4 else 0.1
    lo = np.searchsorted(gyro_t, t0 - 0.02)
    hi = np.searchsorted(gyro_t, t0 + span + 0.02)
    if hi - lo < 2:
        return xyz
    w = gyro_w[lo:hi].mean(axis=0)
    if np.linalg.norm(w) < 1e-3:
        return xyz
    # small-angle rotation about the mean axis, linear in point time
    out = xyz.copy()
    ang = -np.outer(rel - span, w)          # rotate each point back to t_end
    wx, wy, wz = ang[:, 0], ang[:, 1], ang[:, 2]
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    out[:, 0] = x + (wy * z - wz * y)
    out[:, 1] = y + (wz * x - wx * z)
    out[:, 2] = z + (wx * y - wy * x)
    return out


def icp(source_pcd, target_pcd, init):
    result = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, ICP_MAX_CORR_COARSE_M, init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=ICP_ITERS))
    return o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, ICP_MAX_CORR_FINE_M, result.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=ICP_ITERS))


def to_pcd(xyz, voxel):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd.voxel_down_sample(voxel) if voxel else pcd


def find_initial_pose(first_xyz, tmap, traj_path, args):
    """Global search seeded from the 07/07 trajectory.

    The chair is somewhere on the route it was driven along, so candidate
    positions come from the old trajectory rather than the whole map, each
    tried at several yaws.

    Scoring is nearest-neighbour inlier ratio against ONE KD-tree built over
    the whole map, not ICP: registration_icp rebuilds its target tree on
    every call, so scoring ~1000 candidates that way spent minutes without
    producing a single pose. ICP then runs only on the few best candidates.
    """
    from scipy.spatial import cKDTree

    traj = np.loadtxt(traj_path)
    xyz_traj = traj[:, 1:4]
    keep = [0]
    for i in range(1, len(xyz_traj)):
        if np.linalg.norm(xyz_traj[i] - xyz_traj[keep[-1]]) > args.seed_spacing:
            keep.append(i)
    yaws = np.arange(0.0, 2 * math.pi, math.radians(args.seed_yaw_step))

    probe = np.asarray(to_pcd(first_xyz, 0.5).points)
    print("initial pose search: %d positions x %d yaws, %d probe points"
          % (len(keep), len(yaws), len(probe)))
    tree = cKDTree(tmap.points)

    scored = []
    for i in keep:
        for yaw in yaws:
            T = yaw_T(yaw, xyz_traj[i])
            pts = probe @ T[:3, :3].T + T[:3, 3]
            d, _ = tree.query(pts, k=1, distance_upper_bound=0.5, workers=-1)
            scored.append((float(np.mean(np.isfinite(d))), T))
    scored.sort(key=lambda s: -s[0])
    print("  coarse scores: best %.3f, 5th %.3f"
          % (scored[0][0], scored[min(4, len(scored) - 1)][0]))

    src = to_pcd(first_xyz, SCAN_VOXEL_M)
    best = None
    for score, T in scored[:args.seed_refine]:
        local = tmap.local(T[:3, 3][:2])
        if local is None:
            continue
        res = icp(src, local, T)
        if best is None or res.fitness > best[0]:
            best = (res.fitness, res.inlier_rmse, res.transformation)
    if best is None:
        raise SystemExit("initial pose search found no candidates")
    print("initial pose: fitness %.3f rmse %.3f" % (best[0], best[1]))
    if best[0] < MIN_FITNESS:
        raise SystemExit(
            "initial pose fitness %.3f below %.2f - the drive probably "
            "starts outside the 07/07 map coverage" % (best[0], MIN_FITNESS))
    return np.array(best[2]), float(best[0]), float(best[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--traj", required=True)
    ap.add_argument("--out", required=True)
    # topic names differ between today's bags and the normalised 07/07 bag
    ap.add_argument("--lidar-topic", default=LIDAR_TOPIC)
    ap.add_argument("--imu-topic", default=IMU_TOPIC)
    ap.add_argument("--max-seconds", type=float, default=0.0,
                    help="stop early; use to validate before a full run")
    ap.add_argument("--seed-spacing", type=float, default=6.0)
    ap.add_argument("--seed-yaw-step", type=float, default=30.0)
    ap.add_argument("--seed-refine", type=int, default=8,
                    help="top coarse candidates to refine with ICP")
    ap.add_argument("--checkpoint-every", type=int, default=200)
    args = ap.parse_args()

    print("loading map ...")
    t_load = time.time()
    tmap = TiledMap(args.map)
    print("map %d points, %d tiles, %.1fs"
          % (len(tmap.points), len(tmap.tiles), time.time() - t_load))

    gyro_t, gyro_w = read_imu(args.bag, args.imu_topic)
    print("imu samples: %d" % len(gyro_t))

    done = {}
    if os.path.exists(args.out):
        # The whole point of this file is surviving a power cut, and a power
        # cut is exactly what leaves a half-written final record. Parsing it
        # strictly made the failure this recovery exists for the thing that
        # prevented recovery. A damaged line can only ever be the last one,
        # since every earlier line was followed by a completed write.
        damaged = 0
        with open(args.out) as f:
            lines = f.readlines()
        for number, line in enumerate(lines, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
                done[rec["seq"]] = rec
            except (ValueError, KeyError):
                damaged += 1
                if number != len(lines):
                    raise SystemExit(
                        "%s line %d is corrupt but is not the last line - "
                        "refusing to guess which poses are trustworthy"
                        % (args.out, number))
                print("discarding truncated final record (line %d)" % number)
        print("resuming: %d scans already localized%s"
              % (len(done), " (1 partial record dropped)" if damaged else ""))

    pose = None
    prev_delta = np.eye(4)
    if done:
        last = done[max(done)]
        pose = np.eye(4)
        pose[:3, :3] = quat_to_R(last["q"])
        pose[:3, 3] = last["t"]

    out = open(args.out, "a")
    n_ok = n_bad = lost = 0
    t_start = time.time()
    first_stamp = None

    with rosbag.Bag(args.bag) as bag:
        for seq, (_, msg, _) in enumerate(
                bag.read_messages(topics=[args.lidar_topic])):
            stamp = msg.header.stamp.to_sec()
            if first_stamp is None:
                first_stamp = stamp
            if args.max_seconds and stamp - first_stamp > args.max_seconds:
                break
            if seq in done:
                continue

            xyz, inten, rel_t = scan_arrays(msg)
            if len(xyz) < MIN_SCAN_POINTS:
                n_bad += 1
                continue
            xyz = deskew(xyz, rel_t, gyro_t, gyro_w, stamp)
            rng = (xyz * xyz).sum(axis=1)
            xyz = xyz[rng < MAX_SCAN_RANGE_M ** 2]
            if len(xyz) < MIN_SCAN_POINTS:
                n_bad += 1
                continue

            if pose is None:
                if len(xyz) < INIT_MIN_POINTS:
                    continue          # wait for a scan dense enough to trust
                pose, fitness, rmse = find_initial_pose(
                    xyz, tmap, args.traj, args)
                prev_delta = np.eye(4)
                lost = 0

            guess = pose @ prev_delta
            local = tmap.local(guess[:3, 3][:2])
            if local is None:
                print("scan %d: outside map coverage - stopping" % seq)
                break
            res = icp(to_pcd(xyz, SCAN_VOXEL_M), local, guess)
            new_pose = np.array(res.transformation)
            jump = np.linalg.norm(new_pose[:3, 3] - pose[:3, 3])

            good = (res.fitness >= MIN_FITNESS and
                    res.inlier_rmse <= MAX_RMSE_M and jump <= MAX_JUMP_M)
            # metrics belong to whatever actually produced `pose` below
            fitness, rmse, relocalized = res.fitness, res.inlier_rmse, False
            if good:
                prev_delta = np.linalg.inv(pose) @ new_pose
                pose = new_pose
                n_ok += 1
                lost = 0
            else:
                # coast through a short bad patch, but give up and search
                # again rather than extrapolating indefinitely
                pose = guess
                n_bad += 1
                lost += 1
                if lost >= RELOCALIZE_AFTER and len(xyz) >= INIT_MIN_POINTS:
                    print("scan %d: lost for %d scans - relocalizing"
                          % (seq, lost))
                    pose, fitness, rmse = find_initial_pose(
                        xyz, tmap, args.traj, args)
                    prev_delta = np.eye(4)
                    lost = 0
                    relocalized = True

            out.write(json.dumps({
                "seq": seq, "stamp": stamp,
                "t": list(np.round(pose[:3, 3], 4)),
                "q": list(np.round(R_to_quat(pose[:3, :3]), 6)),
                "fitness": round(fitness, 4),
                "rmse": round(rmse, 4),
                "relocalized": relocalized,
                "ok": bool(good)}) + "\n")

            # flush per record: a buffered write means a power cut loses
            # every pose since the last checkpoint, not just the last one.
            # fsync stays on the checkpoint interval - it is the expensive
            # half and losing the page cache needs the whole box to die.
            out.flush()
            if (seq + 1) % args.checkpoint_every == 0:
                os.fsync(out.fileno())
                rate = (seq + 1 - len(done)) / max(time.time() - t_start, 1e-3)
                print("scan %5d  t=%7.1fs  ok=%d bad=%d  fitness=%.3f  "
                      "%.1f scans/s" % (seq, stamp - first_stamp, n_ok, n_bad,
                                        res.fitness, rate))

    out.flush()
    out.close()
    print("\ndone: %d good, %d rejected (%.1f%% good)"
          % (n_ok, n_bad, 100.0 * n_ok / max(n_ok + n_bad, 1)))
    if n_bad > 0.15 * (n_ok + n_bad):
        print("WARNING: high rejection rate - inspect before merging")


if __name__ == "__main__":
    main()
