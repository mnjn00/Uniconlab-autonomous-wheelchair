#!/usr/bin/env python3
"""Read-only preflight for the Mid-360 localization-only stack."""

from __future__ import annotations

import argparse
import hashlib
import math
import sys
import time
from pathlib import Path


EXPECTED_TYPES = {
    "/livox/lidar": "livox_ros_driver2/CustomMsg",
    "/livox/imu": "sensor_msgs/Imu",
    "/Odometry": "nav_msgs/Odometry",
    "/cloud_registered_body": "sensor_msgs/PointCloud2",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_map(path: Path, expected_sha256: str) -> None:
    if path.suffix.lower() not in {".pcd", ".ply"}:
        raise ValueError("map must be .pcd or .ply")
    if len(expected_sha256) != 64:
        raise ValueError("map SHA-256 must contain 64 hexadecimal characters")
    int(expected_sha256, 16)
    observed = sha256_file(path)
    if observed != expected_sha256.lower():
        raise ValueError(f"map SHA-256 mismatch: observed {observed}")


def classify_acceleration_norm(value: float) -> str:
    if not math.isfinite(value) or value <= 0.0:
        return "INVALID"
    if 0.7 <= value <= 1.3:
        return "G_UNITS"
    if 7.0 <= value <= 12.0:
        return "MPS2_UNITS"
    return "UNEXPECTED"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--map-sha256", required=True)
    parser.add_argument("--sample-seconds", type=float, default=2.5)
    args = parser.parse_args()
    validate_map(args.map, args.map_sha256)

    import rosgraph
    import rostopic
    import rospy
    import tf2_ros
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from sensor_msgs.msg import PointCloud2

    if not rosgraph.is_master_online():
        raise RuntimeError("ROS master is not online")
    rospy.init_node("wheelchair_localization_preflight", anonymous=True)

    failures = []
    for topic, expected_type in EXPECTED_TYPES.items():
        observed_type, _, _ = rostopic.get_topic_type(topic, blocking=False)
        if observed_type != expected_type:
            failures.append(
                f"{topic}: expected {expected_type}, observed "
                f"{observed_type or 'MISSING'}"
            )

    imu_count = 0
    imu_first_stamp = None
    imu_last_stamp = None
    imu_norms = []

    def imu_callback(message):
        nonlocal imu_count, imu_first_stamp, imu_last_stamp
        stamp = message.header.stamp.to_sec()
        if stamp <= 0.0 or (
            imu_last_stamp is not None and stamp <= imu_last_stamp
        ):
            failures.append("/livox/imu: zero or non-monotonic source stamp")
            return
        imu_first_stamp = stamp if imu_first_stamp is None else imu_first_stamp
        imu_last_stamp = stamp
        acceleration = message.linear_acceleration
        imu_norms.append(
            math.sqrt(
                acceleration.x**2 + acceleration.y**2 + acceleration.z**2
            )
        )
        imu_count += 1

    subscriber = rospy.Subscriber(
        "/livox/imu", Imu, imu_callback, queue_size=500
    )
    deadline = time.monotonic() + max(0.5, args.sample_seconds)
    while time.monotonic() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.02)
    subscriber.unregister()

    duration = (
        imu_last_stamp - imu_first_stamp
        if imu_first_stamp is not None and imu_last_stamp is not None
        else 0.0
    )
    imu_rate = (imu_count - 1) / duration if duration > 0.0 else 0.0
    if imu_rate < 150.0:
        failures.append(
            f"/livox/imu: measured rate {imu_rate:.1f} Hz, need >=150 Hz"
        )
    mean_norm = sum(imu_norms) / len(imu_norms) if imu_norms else math.nan
    units = classify_acceleration_norm(mean_norm)
    if units not in {"G_UNITS", "MPS2_UNITS"}:
        failures.append(
            f"/livox/imu: unexpected mean acceleration norm {mean_norm:.4f}"
        )

    try:
        odometry = rospy.wait_for_message("/Odometry", Odometry, timeout=2.0)
        if (
            odometry.header.stamp.to_sec() <= 0.0
            or odometry.header.frame_id not in {"camera_init", "odom"}
            or odometry.child_frame_id != "body"
        ):
            failures.append(
                "/Odometry: require nonzero stamp and camera_init/odom -> body"
            )
        orientation = odometry.pose.pose.orientation
        quaternion_norm = math.sqrt(
            orientation.x**2
            + orientation.y**2
            + orientation.z**2
            + orientation.w**2
        )
        if not math.isfinite(quaternion_norm) or abs(quaternion_norm - 1.0) > 1e-3:
            failures.append("/Odometry: pose quaternion is not normalized")
    except Exception as error:
        failures.append(f"/Odometry: no fresh FAST-LIO output: {error}")

    try:
        body_cloud = rospy.wait_for_message(
            "/cloud_registered_body", PointCloud2, timeout=2.0
        )
        if (
            body_cloud.header.stamp.to_sec() <= 0.0
            or body_cloud.header.frame_id != "body"
        ):
            failures.append(
                "/cloud_registered_body: require nonzero stamp and body frame"
            )
        if body_cloud.width * body_cloud.height < 100:
            failures.append("/cloud_registered_body: fewer than 100 points")
    except Exception as error:
        failures.append(
            f"/cloud_registered_body: no fresh FAST-LIO output: {error}"
        )

    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(3.0))
    listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(0.5)
    try:
        transform = tf_buffer.lookup_transform(
            "body", "base_footprint", rospy.Time(0), rospy.Duration(0.5)
        )
        values = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        if not all(math.isfinite(value) for value in values):
            failures.append(
                "body<-base_footprint TF contains non-finite values"
            )
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        translation_norm = math.sqrt(sum(value * value for value in values[:3]))
        if abs(quaternion_norm - 1.0) > 1e-3:
            failures.append(
                "body<-base_footprint TF quaternion is not normalized"
            )
        if not 0.05 <= translation_norm <= 3.0:
            failures.append(
                "body<-base_footprint TF translation is implausible for "
                "the physical Mid-360 mount"
            )
    except Exception as error:
        failures.append(f"body<-base_footprint measured TF missing: {error}")
    del listener

    print(f"map_sha256={args.map_sha256.lower()}")
    print(f"livox_imu_rate_hz={imu_rate:.2f}")
    print(f"livox_imu_mean_acceleration_norm={mean_norm:.6f}")
    print(f"livox_imu_acceleration_units={units}")
    if failures:
        for failure in sorted(set(failures)):
            print(f"FAIL: {failure}", file=sys.stderr)
        return 2
    print("PASS: localization inputs and measured TF are present")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(2)
