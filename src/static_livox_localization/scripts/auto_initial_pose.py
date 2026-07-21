#!/usr/bin/env python3
"""Global initial-pose search: seed the localizer without an operator click.

Accumulates a short odometry-compensated submap, scores candidate poses
sampled along the mapping trajectory by map-inlier fraction (KD-tree), then
publishes the best candidates as /fast_lio_icp/initialpose one at a time.
Each candidate must still pass the localizer's own consensus verification
(VERIFYING -> TRACKING), so a wrong hypothesis is rejected and the next one
is tried automatically.
"""

import argparse
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool

import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft


def load_pcd_xyz(path):
    with open(path, "rb") as f:
        header = b""
        while not header.endswith(b"DATA binary\n"):
            byte = f.read(1)
            if not byte or len(header) > 8192:
                raise RuntimeError("unsupported PCD (need binary)")
            header += byte
        fields = 4 if b"intensity" in header else 3
        data = np.frombuffer(f.read(), dtype=np.float32)
    points = data.reshape(-1, fields)[:, :3]
    return points[np.isfinite(points).all(axis=1)]


def load_candidates(traj_path, spacing):
    rows = np.loadtxt(traj_path)
    positions = rows[:, 1:4]
    keep = [0]
    for index in range(1, len(positions)):
        if np.linalg.norm(positions[index, :2] - positions[keep[-1], :2]) >= spacing:
            keep.append(index)
    candidates = []
    for index in keep:
        x, y, z = positions[index]
        nxt = positions[min(index + 5, len(positions) - 1)]
        heading = np.arctan2(nxt[1] - y, nxt[0] - x)
        candidates.append((x, y, z, heading))
    return candidates


def voxel_downsample(points, size, cap):
    keys = np.floor(points / size).astype(np.int64)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    sampled = points[np.sort(unique_idx)]
    if len(sampled) > cap:
        sampled = sampled[np.random.RandomState(0).choice(len(sampled), cap, False)]
    return sampled


class SubmapCollector:
    def __init__(self, window_s):
        self.window_s = window_s
        self.odom = []
        self.clouds = []
        self.odom_sub = rospy.Subscriber("/Odometry", Odometry, self.on_odom, queue_size=100)
        self.cloud_sub = rospy.Subscriber(
            "/cloud_registered_body", PointCloud2, self.on_cloud, queue_size=10)

    def on_odom(self, message):
        q = message.pose.pose.orientation
        p = message.pose.pose.position
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[:3, 3] = (p.x, p.y, p.z)
        self.odom.append((message.header.stamp.to_sec(), T))
        self.odom = self.odom[-400:]

    def on_cloud(self, message):
        pts = np.array(list(pc2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)
        self.clouds.append((message.header.stamp.to_sec(), pts))
        cutoff = message.header.stamp.to_sec() - self.window_s - 1.0
        self.clouds = [c for c in self.clouds if c[0] >= cutoff]

    def build(self):
        if not self.clouds or not self.odom:
            return None
        newest = self.clouds[-1][0]
        odom_t = np.array([t for t, _ in self.odom])
        ref = None
        merged = []
        for stamp, pts in self.clouds:
            if newest - stamp > self.window_s or len(pts) == 0:
                continue
            k = int(np.argmin(np.abs(odom_t - stamp)))
            if abs(odom_t[k] - stamp) > 0.12:
                continue
            T = self.odom[k][1]
            if ref is None or stamp == newest:
                ref = T
            hom = np.hstack([pts, np.ones((len(pts), 1), np.float32)])
            merged.append((T @ hom.T).T[:, :3])
        if not merged or ref is None:
            return None
        world = np.vstack(merged)
        inv = np.linalg.inv(ref)
        hom = np.hstack([world, np.ones((len(world), 1))])
        return (inv @ hom.T).T[:, :3].astype(np.float32)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True)
    parser.add_argument("--traj", required=True)
    parser.add_argument("--spacing", type=float, default=3.0)
    parser.add_argument("--inlier-radius", type=float, default=0.45)
    parser.add_argument("--min-score", type=float, default=0.25)
    parser.add_argument("--top", type=int, default=4)
    parser.add_argument("--window-s", type=float, default=2.0)
    parser.add_argument("--max-range", type=float, default=25.0)
    parser.add_argument("--verify-timeout", type=float, default=20.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])

    rospy.init_node("auto_initial_pose")
    rospy.loginfo("loading map and trajectory")
    map_points = load_pcd_xyz(args.map)
    tree = cKDTree(map_points)
    candidates = load_candidates(args.traj, args.spacing)
    rospy.loginfo("%d trajectory candidates", len(candidates))

    collector = SubmapCollector(args.window_s)
    deadline = rospy.Time.now() + rospy.Duration(30.0)
    submap = None
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        rospy.sleep(0.5)
        submap = collector.build()
        if submap is not None and len(submap) > 2000 and len(collector.clouds) >= 10:
            break
    if submap is None or len(submap) < 500:
        rospy.logerr("no usable submap from /cloud_registered_body")
        return 2
    ranges = np.linalg.norm(submap[:, :2], axis=1)
    submap = submap[ranges < args.max_range]
    sample = voxel_downsample(submap, 0.4, 1800)
    rospy.loginfo("submap sample: %d points", len(sample))

    yaw_offsets = (0.0, np.pi, np.pi / 4, -np.pi / 4, 3 * np.pi / 4, -3 * np.pi / 4)
    scored = []
    ones = np.ones((len(sample), 1), np.float32)
    for x, y, z, heading in candidates:
        for offset in yaw_offsets:
            yaw = heading + offset
            c, s = np.cos(yaw), np.sin(yaw)
            R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], np.float32)
            world = sample @ R.T + np.array([x, y, z], np.float32)
            dists, _ = tree.query(world, k=1, distance_upper_bound=args.inlier_radius)
            score = float(np.isfinite(dists).mean())
            scored.append((score, x, y, z, yaw))
    scored.sort(key=lambda item: -item[0])

    rospy.loginfo("top candidates:")
    for score, x, y, z, yaw in scored[:6]:
        rospy.loginfo("  score=%.3f  (%.1f, %.1f, %.1f) yaw=%.0fdeg",
                      score, x, y, z, np.degrees(yaw))

    if args.dry_run:
        return 0
    if scored[0][0] < args.min_score:
        rospy.logerr("best score %.3f below threshold %.2f - not seeding",
                     scored[0][0], args.min_score)
        return 3

    state = {"message": ""}

    def on_diag(message):
        for status in message.status:
            if status.name == "fast_lio_icp":
                state["message"] = status.message

    rospy.Subscriber("/fast_lio_icp/localization_diagnostics",
                     DiagnosticArray, on_diag, queue_size=5)
    seed_pub = rospy.Publisher("/fast_lio_icp/initialpose",
                               PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(0.5)
    enable = rospy.ServiceProxy("/fast_lio_icp/enable_auto_correction", SetBool)

    for rank, (score, x, y, z, yaw) in enumerate(scored[:args.top]):
        if score < args.min_score:
            break
        rospy.loginfo("trying candidate %d: score=%.3f (%.1f, %.1f) yaw=%.0f",
                      rank + 1, score, x, y, np.degrees(yaw))
        seed = PoseWithCovarianceStamped()
        seed.header.frame_id = "map"
        seed.header.stamp = rospy.Time.now()
        seed.pose.pose.position.x = x
        seed.pose.pose.position.y = y
        seed.pose.pose.position.z = z
        q = tft.quaternion_from_euler(0, 0, yaw)
        seed.pose.pose.orientation.x = q[0]
        seed.pose.pose.orientation.y = q[1]
        seed.pose.pose.orientation.z = q[2]
        seed.pose.pose.orientation.w = q[3]
        seed_pub.publish(seed)
        rospy.sleep(1.0)
        rospy.wait_for_service("/fast_lio_icp/enable_auto_correction", timeout=10.0)
        enable(True)
        verify_deadline = rospy.Time.now() + rospy.Duration(args.verify_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < verify_deadline:
            if state["message"] == "TRACKING":
                rospy.loginfo("initialized: candidate %d verified (TRACKING)", rank + 1)
                return 0
            rospy.sleep(0.5)
        rospy.logwarn("candidate %d failed verification, trying next", rank + 1)
    rospy.logerr("no candidate passed verification")
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
