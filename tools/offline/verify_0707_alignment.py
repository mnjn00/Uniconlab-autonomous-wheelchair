#!/usr/bin/env python3
"""Check the 07/07 raw scans against GLIM's optimised trajectory and map.

Two things are established before any of today's data is trusted:

  1. scans + traj_lidar.txt really do reproduce the published map, i.e.
     the trajectory is the lidar pose in the map frame and the timestamps
     pair up. If this fails, the whole "reuse GLIM's poses" shortcut is
     invalid.
  2. how closely plain point-to-plane ICP agrees with GLIM's loop-closed
     solution. GLIM is the better estimator, so the delta measured here is
     an upper bound on the tracking error to expect on today's bags, where
     no ground truth exists.

Reports per sample: fitness/RMSE at the GLIM pose, and how far ICP pulls
away from it. A small pull means ICP is consistent with the reference.
"""

import argparse
import math

import numpy as np
import open3d as o3d
import rosbag


def detect_layout(msg):
    """Field offsets by name, plus which field carries per-point time."""
    off = {f.name: (f.offset, f.datatype) for f in msg.fields}
    for name in ("t", "timestamp", "time"):
        if name in off:
            return off, name
    return off, None


def scan_xyz(msg, layout, time_name):
    off, _ = layout, None
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    xo = off["x"][0]
    xyz = raw[:, xo:xo + 12].copy().view(np.float32).reshape(-1, 3)
    xyz = xyz.astype(np.float64)
    ok = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 1e-6)
    return xyz[ok]


def slerp(q0, q1, u):
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th0 = math.acos(d)
    th = th0 * u
    s0 = math.sin(th0 - th) / math.sin(th0)
    s1 = math.sin(th) / math.sin(th0)
    return q0 * s0 + q1 * s1


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_at(traj, t):
    ts = traj[:, 0]
    if t <= ts[0] or t >= ts[-1]:
        return None
    i = int(np.searchsorted(ts, t)) - 1
    u = (t - ts[i]) / (ts[i + 1] - ts[i])
    T = np.eye(4)
    T[:3, 3] = traj[i, 1:4] * (1 - u) + traj[i + 1, 1:4] * u
    T[:3, :3] = quat_to_R(slerp(traj[i, 4:8], traj[i + 1, 4:8], u))
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--lidar-topic", default="/sensors/lidar/points")
    ap.add_argument("--map", required=True)
    ap.add_argument("--traj", required=True)
    ap.add_argument("--samples", type=int, default=20)
    ap.add_argument("--voxel", type=float, default=0.25)
    ap.add_argument("--max-corr", type=float, default=0.5)
    args = ap.parse_args()

    print("loading map ...")
    m = o3d.io.read_point_cloud(args.map)
    m.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.6, max_nn=30))
    map_pts = np.asarray(m.points)
    print("map %d points" % len(map_pts))

    traj = np.loadtxt(args.traj)
    print("traj %d poses, %.1f..%.1f" % (len(traj), traj[0, 0], traj[-1, 0]))

    with rosbag.Bag(args.bag) as bag:
        total = bag.get_message_count(topic_filters=[args.lidar_topic])
        step = max(total // args.samples, 1)
        print("bag %d scans, sampling every %d\n" % (total, step))

        print("%6s %12s  %7s %7s   %8s %8s %7s" % (
            "scan", "stamp", "fit@GT", "rmse@GT", "ICPmove", "fit@ICP", "yaw°"))
        moves, fits_gt, fits_icp = [], [], []
        layout = None
        for seq, (_, msg, _) in enumerate(
                bag.read_messages(topics=[args.lidar_topic])):
            if seq % step or seq == 0:
                continue
            if layout is None:
                layout, tname = detect_layout(msg)
                print("(point fields: %s, time field: %s)\n"
                      % (sorted(layout), tname))
            stamp = msg.header.stamp.to_sec()
            T_gt = pose_at(traj, stamp)
            if T_gt is None:
                continue
            xyz = scan_xyz(msg, layout, None)
            if len(xyz) < 500:
                continue
            src = o3d.geometry.PointCloud()
            src.points = o3d.utility.Vector3dVector(xyz)
            src = src.voxel_down_sample(args.voxel)

            # local crop so evaluation is not dominated by the far map
            c = T_gt[:3, 3]
            sel = np.flatnonzero(
                np.max(np.abs(map_pts - c), axis=1) < 40.0)
            if len(sel) < 5000:
                continue
            tgt = m.select_by_index(sel)

            ev = o3d.pipelines.registration.evaluate_registration(
                src, tgt, args.max_corr, T_gt)
            res = o3d.pipelines.registration.registration_icp(
                src, tgt, args.max_corr, T_gt,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=30))
            T_icp = np.array(res.transformation)
            move = float(np.linalg.norm(T_icp[:3, 3] - T_gt[:3, 3]))
            dR = T_gt[:3, :3].T @ T_icp[:3, :3]
            dyaw = math.degrees(math.atan2(dR[1, 0], dR[0, 0]))

            moves.append(move)
            fits_gt.append(ev.fitness)
            fits_icp.append(res.fitness)
            print("%6d %12.2f  %7.3f %7.3f   %8.3f %8.3f %+7.2f"
                  % (seq, stamp, ev.fitness, ev.inlier_rmse,
                     move, res.fitness, dyaw))

    if not moves:
        raise SystemExit("no samples evaluated")
    moves = np.array(moves)
    print("\nfitness at GLIM pose : mean %.3f  min %.3f"
          % (np.mean(fits_gt), np.min(fits_gt)))
    print("fitness after ICP    : mean %.3f" % np.mean(fits_icp))
    print("ICP pull from GLIM   : mean %.3f m  median %.3f m  max %.3f m"
          % (moves.mean(), np.median(moves), moves.max()))
    if np.mean(fits_gt) < 0.5:
        print("\nVERDICT: scans do NOT reproduce the map at the GLIM poses - "
              "the trajectory/scan pairing or frame assumption is wrong")
    elif np.median(moves) < 0.10:
        print("\nVERDICT: ICP agrees with GLIM to under 10 cm - safe to use "
              "ICP tracking for today's bags")
    else:
        print("\nVERDICT: ICP drifts %.2f m from GLIM - tighten registration "
              "before trusting it on today's bags" % np.median(moves))


if __name__ == "__main__":
    main()
