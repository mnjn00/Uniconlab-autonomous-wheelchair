#!/usr/bin/env python3
"""Capture perception summary/boxes from one source cycle plus diagnostics."""

import argparse
import json
from pathlib import Path

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def stamp_value(stamp):
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def message_yaml(message):
    """ROS message __str__ is the same YAML form emitted by rostopic echo."""
    return str(message) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", required=True)
    parser.add_argument("--boxes", required=True)
    parser.add_argument("--diagnostics", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    rospy.init_node("capture_shadow_snapshot", anonymous=True)

    summaries = {}
    boxes = {}
    diagnostics = {}

    def on_summary(message):
        payload = json.loads(message.data)
        summaries[round(float(payload["stamp"]), 6)] = message

    def on_boxes(message):
        if not message.markers:
            return
        source = stamp_value(message.markers[0].header.stamp)
        if all(abs(stamp_value(marker.header.stamp) - source) < 1e-6
               for marker in message.markers):
            boxes[round(source, 6)] = message

    def on_diagnostics(message):
        source = stamp_value(message.header.stamp)
        diagnostics[round(source, 6)] = message

    rospy.Subscriber("/perception/objects_summary", String, on_summary)
    rospy.Subscriber("/perception/dynamic_boxes", MarkerArray, on_boxes)
    rospy.Subscriber(
        "/fast_lio_icp/localization_diagnostics",
        DiagnosticArray,
        on_diagnostics,
    )
    deadline = rospy.Time.now() + rospy.Duration(args.timeout)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        shared = summaries.keys() & boxes.keys() & diagnostics.keys()
        if shared:
            source = max(shared)
            Path(args.summary).write_text(
                message_yaml(summaries[source]),
                encoding="utf-8",
            )
            Path(args.boxes).write_text(
                message_yaml(boxes[source]),
                encoding="utf-8",
            )
            Path(args.diagnostics).write_text(
                message_yaml(diagnostics[source]),
                encoding="utf-8",
            )
            return
        rate.sleep()
    raise RuntimeError("timed out waiting for one coherent shadow cycle")


if __name__ == "__main__":
    main()
