#!/usr/bin/env python3
"""Publish conditioned people for a namespaced, command-sunk CoHAN shadow."""

import importlib
import json
import math
import sys
from pathlib import Path

import rospy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import String

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

cluster_guard = importlib.import_module("cluster_guard")
object_box = cluster_guard.object_box
object_geometry_valid = cluster_guard.object_geometry_valid
object_motion = cluster_guard.object_motion
parse_summary = cluster_guard.parse_summary

human_aware_shadow = importlib.import_module("human_aware_shadow")
HumanAwareConditioner = human_aware_shadow.HumanAwareConditioner
PersonObservation = human_aware_shadow.PersonObservation
RobotPose2D = human_aware_shadow.RobotPose2D
ShadowDecision = human_aware_shadow.ShadowDecision
finite_or_nan = human_aware_shadow.finite_or_nan
to_cohan_agent = human_aware_shadow.to_cohan_agent

PERSON_LABEL = "person"


def person_observations(summary):
    """Parse directly observed people from one validated producer cycle."""
    people = []
    for item in summary.objects:
        if str(item.get("class", "")).strip().lower() != PERSON_LABEL:
            continue
        box = object_box(item)
        geometry_valid = object_geometry_valid(item)
        if box is None:
            forward_m = lateral_m = half_length_m = half_width_m = 0.0
        else:
            forward_m, lateral_m, half_length_m, half_width_m = box
        track_id = item.get("id")
        identity_valid = isinstance(track_id, int) and not isinstance(
            track_id, bool)
        people.append(PersonObservation(
            track_id=track_id if identity_valid else -1,
            observed_stamp_s=summary.stamp_s,
            motion=object_motion(item),
            speed_mps=finite_or_nan(item.get("speed_mps")),
            forward_m=forward_m,
            lateral_m=lateral_m,
            half_length_m=half_length_m,
            half_width_m=half_width_m,
            directly_observed=identity_valid,
            geometry_valid=geometry_valid,
        ))
    return people


class HumanAwareShadowNode:
    """ROS boundary; publishes advisory people/status and no motion command."""

    def __init__(self):
        from cohan_msgs.msg import (
            AgentType,
            TrackedAgent,
            TrackedAgents,
            TrackedSegment,
            TrackedSegmentType,
        )

        rospy.init_node("human_aware_shadow")
        self.AgentType = AgentType
        self.TrackedAgent = TrackedAgent
        self.TrackedAgents = TrackedAgents
        self.TrackedSegment = TrackedSegment
        self.TrackedSegmentType = TrackedSegmentType
        self.global_frame = str(rospy.get_param("~global_frame", "map"))
        self.base_frame = str(
            rospy.get_param("~base_frame", "base_footprint")
        )
        self.tf_broadcaster = (
            tf2_ros.TransformBroadcaster()
            if bool(rospy.get_param("~broadcast_robot_tf", False))
            else None
        )
        self.conditioner = HumanAwareConditioner()
        self.robot_pose = None
        self.localization_tracking = False
        self.tracked_agents_pub = rospy.Publisher(
            "/human_aware_shadow/tracked_agents",
            TrackedAgents,
            queue_size=1,
        )
        self.status_pub = rospy.Publisher(
            "/human_aware_shadow/status",
            String,
            queue_size=2,
        )
        rospy.Subscriber(
            "/fast_lio_icp/pose",
            PoseWithCovarianceStamped,
            self.on_pose,
            queue_size=5,
        )
        rospy.Subscriber(
            "/fast_lio_icp/localization_diagnostics",
            DiagnosticArray,
            self.on_diagnostics,
            queue_size=5,
        )
        rospy.Subscriber(
            "/perception/objects_summary",
            String,
            self.on_summary,
            queue_size=2,
        )

    def on_pose(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        self.robot_pose = RobotPose2D(
            x_m=float(position.x),
            y_m=float(position.y),
            yaw_rad=yaw,
        )
        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = message.header.stamp
            transform.header.frame_id = self.global_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = position.x
            transform.transform.translation.y = position.y
            transform.transform.translation.z = position.z
            transform.transform.rotation = orientation
            self.tf_broadcaster.sendTransform(transform)

    def on_diagnostics(self, message):
        self.localization_tracking = any(
            status.name == "fast_lio_icp"
            and status.message == "TRACKING"
            for status in message.status
        )

    def tracked_agents_message(self, stamp_s, payload=None):
        message = self.TrackedAgents()
        message.header.stamp = rospy.Time.from_sec(stamp_s)
        message.header.frame_id = self.global_frame
        if payload is None:
            return message
        segment = self.TrackedSegment()
        segment.type = self.TrackedSegmentType.TORSO
        segment.pose.pose.position.x = payload.x_m
        segment.pose.pose.position.y = payload.y_m
        segment.pose.pose.orientation.w = 1.0
        segment.pose.covariance[0] = payload.position_variance
        segment.pose.covariance[7] = payload.position_variance
        segment.pose.covariance[35] = 1.0e3
        segment.twist.twist.linear.x = payload.vx_mps
        segment.twist.twist.linear.y = payload.vy_mps
        agent = self.TrackedAgent()
        agent.track_id = payload.track_id
        agent.state = payload.state
        agent.type = self.AgentType.HUMAN
        agent.name = f"person_{payload.track_id}"
        agent.segments.append(segment)
        message.agents.append(agent)
        return message

    def publish_stop_required(self, stamp_s, reason):
        self.tracked_agents_pub.publish(
            self.tracked_agents_message(stamp_s))
        self.status_pub.publish(String(data=json.dumps({
            "decision": ShadowDecision.STOP_REQUIRED.value,
            "reason": reason,
            "stamp": stamp_s,
        }, sort_keys=True)))

    def on_summary(self, message):
        try:
            summary = parse_summary(message.data)
        except ValueError as error:
            self.publish_stop_required(
                rospy.Time.now().to_sec(), f"MALFORMED_SUMMARY:{error}")
            return
        people = person_observations(summary)
        tracking = (
            summary.usable
            and self.localization_tracking
            and self.robot_pose is not None
        )
        snapshot = self.conditioner.update(
            summary.stamp_s,
            people,
            localization_tracking=tracking,
        )
        payload = None
        if (
                snapshot.decision is ShadowDecision.BYPASS_COMMITTED
                and len(people) == 1
                and self.robot_pose is not None):
            payload = to_cohan_agent(
                people[0], self.robot_pose, self.global_frame)
        self.tracked_agents_pub.publish(
            self.tracked_agents_message(summary.stamp_s, payload))
        self.status_pub.publish(String(data=json.dumps({
            "decision": snapshot.decision.value,
            "evidence_s": round(snapshot.evidence_s, 3),
            "person_count": len(people),
            "stamp": summary.stamp_s,
            "track_id": snapshot.track_id,
        }, sort_keys=True)))


def main():
    HumanAwareShadowNode()
    rospy.spin()


if __name__ == "__main__":
    main()
