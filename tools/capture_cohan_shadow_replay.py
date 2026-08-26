#!/usr/bin/env python3
"""Capture coherent advisory CoHAN/HATEB replay evidence to one JSON file."""

import argparse
import json
import sys
from pathlib import Path

import rospy
from cohan_msgs.msg import TrackedAgents
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import Empty, String

ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    from human_aware_shadow import ShadowTrajectoryValidator
    from route_mask import RouteMask
    from safety_band import SafetyBand
finally:
    sys.path.pop(0)


def stamp_s(stamp):
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


class ReplayCapture:
    """Mutable ROS subscriber sink; accumulation is its purpose."""

    def __init__(self, output_path, band_path, mask_path):
        self.output_path = Path(output_path)
        self.validator = ShadowTrajectoryValidator(
            SafetyBand(band_path),
            RouteMask(mask_path),
        )
        self.evidence = {
            "statuses": [],
            "summaries": [],
            "tracked_agents": [],
            "velocity_proposals": [],
            "local_plans": [],
        }
        self.written = False
        rospy.on_shutdown(self.write)
        rospy.Subscriber(
            "/perception/objects_summary",
            String,
            self.on_summary,
            queue_size=100,
        )
        rospy.Subscriber(
            "/human_aware_shadow/status",
            String,
            self.on_status,
            queue_size=100,
        )
        rospy.Subscriber(
            "/human_aware_shadow/tracked_agents",
            TrackedAgents,
            self.on_agents,
            queue_size=100,
        )
        rospy.Subscriber(
            "/human_aware_shadow/velocity_proposal",
            Twist,
            self.on_proposal,
            queue_size=100,
        )
        rospy.Subscriber(
            "/human_aware_shadow/move_base/HATebLocalPlannerROS/local_plan",
            RosPath,
            self.on_local_plan,
            queue_size=100,
        )
        rospy.Subscriber(
            "/human_aware_shadow/replay_done",
            Empty,
            self.on_done,
            queue_size=1,
        )

    def on_summary(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        objects = payload.get("objects")
        if not isinstance(objects, list):
            return
        people = [
            {"id": item.get("id"), "motion": item.get("motion")}
            for item in objects
            if str(item.get("class", "")).strip().lower() == "person"
        ]
        self.evidence["summaries"].append({
            "stamp": float(payload.get("stamp", 0.0)),
            "people": people,
        })

    def on_status(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            self.evidence["statuses"].append(payload)

    def on_agents(self, message):
        track_ids = [int(agent.track_id) for agent in message.agents]
        if not track_ids:
            return
        self.evidence["tracked_agents"].append({
            "stamp": stamp_s(message.header.stamp),
            "frame_id": message.header.frame_id,
            "track_ids": track_ids,
        })

    def on_proposal(self, message):
        if abs(message.linear.x) + abs(message.angular.z) <= 1e-9:
            return
        self.evidence["velocity_proposals"].append({
            "stamp": rospy.Time.now().to_sec(),
            "linear_x": float(message.linear.x),
            "angular_z": float(message.angular.z),
        })

    def on_local_plan(self, message):
        points = tuple(
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
        )
        validation = self.validator.validate(points)
        self.evidence["local_plans"].append({
            "stamp": stamp_s(message.header.stamp),
            "validation": validation.value,
            "point_count": len(points),
        })

    def write(self):
        if self.written:
            return
        self.output_path.write_text(
            json.dumps(self.evidence, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        self.written = True

    def on_done(self, _message):
        self.write()
        rospy.signal_shutdown("replay complete")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--band", required=True)
    parser.add_argument("--drivable-mask", required=True)
    args = parser.parse_args()
    rospy.init_node("capture_cohan_shadow_replay")
    ReplayCapture(args.output, args.band, args.drivable_mask)
    rospy.spin()


if __name__ == "__main__":
    main()
