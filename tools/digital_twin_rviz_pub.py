#!/usr/bin/env python3
"""Digital twin publisher for RViz.

Loads the merged PCD map and the 0727 route, drives a virtual
wheelchair along the route, and publishes:
  /map_cloud           full merged map (static, published once)
  /cloud_registered_body  synthetic MID360 scan at the current pose
  /Odometry            synthetic odometry
  /fast_lio_icp/pose   synthetic localization pose
  /digital_twin/marker wheelchair footprint + heading arrow

Run inside the Docker container alongside RViz.
"""

import json
import math
import struct
import sys

import numpy as np
import rospy
from geometry_msgs.msg import (PoseWithCovarianceStamped, Twist,
                               Quaternion)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations as tft

SENSOR_HEIGHT = 0.725
VERT_FOV_MIN = math.radians(-7.0)
VERT_FOV_MAX = math.radians(52.0)
SCAN_RADIUS = 12.0
SPEED = 0.6
HZ = 10.0


def read_pcd_xyz(path):
    """Minimal ASCII/binary PCD reader -> (N,3) float32."""
    with open(path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline().decode("ascii", errors="replace").strip()
            header_lines.append(line)
            if line.startswith("DATA"):
                break
        fields, size, typ, count = [], [], [], []
        n_points = 0
        data_type = "ascii"
        for line in header_lines:
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("SIZE"):
                size = [int(x) for x in line.split()[1:]]
            elif line.startswith("TYPE"):
                typ = line.split()[1:]
            elif line.startswith("COUNT"):
                count = [int(x) for x in line.split()[1:]]
            elif line.startswith("POINTS"):
                n_points = int(line.split()[1])
            elif line.startswith("DATA"):
                data_type = line.split()[1]

        if data_type == "ascii":
            pts = []
            for _ in range(n_points):
                vals = f.readline().split()
                pts.append([float(vals[0]), float(vals[1]), float(vals[2])])
            return np.array(pts, dtype=np.float32)

        # binary
        point_size = sum(s * c for s, c in zip(size, count))
        raw = f.read(n_points * point_size)
        pts = np.zeros((n_points, 3), dtype=np.float32)
        # assume F F F F (x y z intensity), all float32
        for i in range(n_points):
            offset = i * point_size
            x, y, z = struct.unpack_from("<fff", raw, offset)
            pts[i] = [x, y, z]
        return pts


def make_cloud(header, pts):
    """(N,3) float32 -> PointCloud2."""
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(pts)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(pts)
    msg.is_dense = True
    msg.data = pts.astype(np.float32).tobytes()
    return msg


def synthetic_scan(map_pts, pos, yaw):
    rel = map_pts - pos
    c, s = math.cos(-yaw), math.sin(-yaw)
    x = rel[:, 0] * c - rel[:, 1] * s
    y = rel[:, 0] * s + rel[:, 1] * c
    z = rel[:, 2]
    r = np.sqrt(x * x + y * y)
    mask = r < SCAN_RADIUS
    z_s = z + SENSOR_HEIGHT
    elev = np.arctan2(z_s, np.maximum(r, 0.01))
    mask &= (elev > VERT_FOV_MIN) & (elev < VERT_FOV_MAX) & (z_s > -0.05)
    scan = np.column_stack([x[mask], y[mask], z[mask]])
    if len(scan) > 24000:
        idx = np.random.choice(len(scan), 24000, replace=False)
        scan = scan[idx]
    return scan


def interpolate_route(wps, yaws, step_m=0.2):
    dense_xy, dense_yaw = [], []
    for i in range(len(wps) - 1):
        p0, p1 = wps[i], wps[i + 1]
        seg = np.linalg.norm(p1[:2] - p0[:2])
        n = max(int(seg / step_m), 1)
        for j in range(n):
            t = j / n
            xy = p0[:2] + t * (p1[:2] - p0[:2])
            z = p0[2] + t * (p1[2] - p0[2])
            dense_xy.append([xy[0], xy[1], z])
            dense_yaw.append(yaws[i] + t * (yaws[i + 1] - yaws[i]))
    dense_xy.append(wps[-1].tolist())
    dense_yaw.append(yaws[-1])
    return np.array(dense_xy), np.array(dense_yaw)


def main():
    rospy.init_node("digital_twin_pub")
    map_path = rospy.get_param("~map", "/backtest/map.pcd")
    route_path = rospy.get_param("~route", "/backtest/route.json")

    rospy.loginfo("Loading map: %s", map_path)
    map_pts = read_pcd_xyz(map_path)
    rospy.loginfo("Map: %d points", len(map_pts))

    with open(route_path) as f:
        rdata = json.load(f)
    wps = np.array([[w["x"], w["y"], w.get("z", 0.0)]
                     for w in rdata["waypoints"]])
    yaws = np.array([math.radians(w.get("yaw_deg", 0.0))
                      for w in rdata["waypoints"]])
    dense_xy, dense_yaw = interpolate_route(wps, yaws)
    segs = np.linalg.norm(np.diff(dense_xy[:, :2], axis=0), axis=1)
    cumdist = np.concatenate([[0, ], np.cumsum(segs)])
    total_dist = cumdist[-1]
    total_frames = len(dense_xy)
    rospy.loginfo("Route: %d wp -> %d dense frames, %.0f m",
                  len(wps), total_frames, total_dist)

    map_pub = rospy.Publisher("/map_cloud", PointCloud2,
                              queue_size=1, latch=True)
    scan_pub = rospy.Publisher("/cloud_registered_body", PointCloud2,
                               queue_size=1)
    odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=1)
    pose_pub = rospy.Publisher("/fast_lio_icp/pose",
                               PoseWithCovarianceStamped, queue_size=1)
    marker_pub = rospy.Publisher("/digital_twin/marker", MarkerArray,
                                 queue_size=1)

    # publish full map once
    hdr = Header(stamp=rospy.Time.now(), frame_id="map")
    map_pub.publish(make_cloud(hdr, map_pts))
    rospy.loginfo("Map published")

    rate = rospy.Rate(HZ)
    frame = 0
    step_per_frame = SPEED / HZ

    while not rospy.is_shutdown():
        dist = frame * step_per_frame
        if dist > total_dist:
            rospy.loginfo("Route complete (%.0f m). Looping.", total_dist)
            frame = 0
            dist = 0.0

        idx = np.searchsorted(cumdist, dist, side="right") - 1
        idx = min(idx, total_frames - 2)
        frac = (dist - cumdist[idx]) / max(segs[idx], 1e-6)
        pos = dense_xy[idx] + frac * (dense_xy[idx + 1] - dense_xy[idx])
        yaw = dense_yaw[idx] + frac * (dense_yaw[idx + 1] - dense_yaw[idx])
        now = rospy.Time.now()

        # synthetic scan (body frame)
        scan = synthetic_scan(map_pts, pos, yaw)
        scan_hdr = Header(stamp=now, frame_id="body")
        scan_pub.publish(make_cloud(scan_hdr, scan))

        # odometry
        q = tft.quaternion_from_euler(0, 0, yaw)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "body"
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation = Quaternion(*q)
        odom.twist.twist.linear.x = SPEED
        odom_pub.publish(odom)

        # localization pose
        ps = PoseWithCovarianceStamped()
        ps.header.stamp = now
        ps.header.frame_id = "map"
        ps.pose.pose.position.x = pos[0]
        ps.pose.pose.position.y = pos[1]
        ps.pose.pose.position.z = pos[2]
        ps.pose.pose.orientation = Quaternion(*q)
        pose_pub.publish(ps)

        # markers: wheelchair box + heading arrow
        ma = MarkerArray()
        box = Marker()
        box.header.frame_id = "map"
        box.header.stamp = now
        box.ns = "chair"
        box.id = 0
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x = pos[0]
        box.pose.position.y = pos[1]
        box.pose.position.z = pos[2] + 0.3
        box.pose.orientation = Quaternion(*q)
        box.scale.x, box.scale.y, box.scale.z = 1.1, 0.65, 0.6
        box.color.r, box.color.g, box.color.b, box.color.a = 0.9, 0.15, 0.1, 0.8
        ma.markers.append(box)

        arrow = Marker()
        arrow.header.frame_id = "map"
        arrow.header.stamp = now
        arrow.ns = "heading"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = pos[0]
        arrow.pose.position.y = pos[1]
        arrow.pose.position.z = pos[2] + 0.7
        arrow.pose.orientation = Quaternion(*q)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 1.5, 0.15, 0.15
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = \
            0.1, 0.9, 0.2, 0.9
        ma.markers.append(arrow)
        marker_pub.publish(ma)

        pct = 100 * dist / total_dist
        if frame % 50 == 0:
            rospy.loginfo("%.0f%%  wp~%d/%d  (%.1f,%.1f)  scan:%d",
                          pct, idx, total_frames, pos[0], pos[1], len(scan))
        frame += 1
        rate.sleep()


if __name__ == "__main__":
    main()