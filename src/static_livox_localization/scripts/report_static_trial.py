#!/usr/bin/env python3
"""Record stationary pose/diagnostic evidence without publishing any topic."""

import argparse
import json
import math
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped


class Recorder:
    def __init__(self):
        self.poses = []
        self.states = []
        rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped, self.pose, queue_size=100)
        rospy.Subscriber("/fast_lio_icp/localization_diagnostics", DiagnosticArray, self.diagnostic, queue_size=100)

    def pose(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.poses.append({"t": message.header.stamp.to_sec(), "x": p.x, "y": p.y, "z": p.z, "yaw": yaw})

    def diagnostic(self, message):
        for status in message.status:
            if status.name == "fast_lio_icp":
                values = {item.key: item.value for item in status.values}
                self.states.append({"t": message.header.stamp.to_sec(), "state": values.get("raw_state"),
                                    "fitness": values.get("fitness"), "inlier_ratio": values.get("inlier_ratio")})


def angular_distance(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--trial-id", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("static_localization_reporter", anonymous=True, disable_signals=True)
    recorder = Recorder()
    deadline = time.monotonic() + args.duration
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        rospy.sleep(0.1)
    report = {"trial_id": args.trial_id, "duration_s": args.duration, "poses": recorder.poses,
              "diagnostics": recorder.states, "pose_count": len(recorder.poses)}
    if len(recorder.poses) >= 2:
        first, last = recorder.poses[0], recorder.poses[-1]
        report["drift_m"] = math.hypot(last["x"] - first["x"], last["y"] - first["y"])
        report["yaw_drift_deg"] = math.degrees(angular_distance(last["yaw"], first["yaw"]))
    report["raw_lost_observed"] = any(item["state"] == "RAW_LOST" for item in recorder.states)
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(args.output)
    return 0 if recorder.poses else 2


if __name__ == "__main__":
    raise SystemExit(main())
