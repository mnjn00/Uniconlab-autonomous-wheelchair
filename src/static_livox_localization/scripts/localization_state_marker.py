#!/usr/bin/env python3
"""Color the estimated wheelchair pose by localization state."""

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker


COLORS = {
    "MANUAL_ALIGN": (0.0, 1.0, 1.0),
    "VERIFYING": (1.0, 0.0, 1.0),
    "TRACKING": (0.0, 1.0, 0.0),
    "DEGRADED": (1.0, 1.0, 0.0),
    "LOST": (1.0, 0.0, 0.0),
    "WAITING_INITIALIZATION": (0.2, 0.5, 1.0),
}


class LocalizationStateMarker:
    def __init__(self):
        self.pose = None
        self.state = "WAITING_INITIALIZATION"
        self.publisher = rospy.Publisher(
            "/fast_lio_icp/state_marker", Marker, queue_size=1, latch=True
        )
        self.footprint_publisher = rospy.Publisher(
            "/fast_lio_icp/wheelchair_footprint_marker", Marker, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/fast_lio_icp/pose", PoseWithCovarianceStamped,
            self.pose_callback, queue_size=1
        )
        rospy.Subscriber(
            "/fast_lio_icp/localization_diagnostics", DiagnosticArray,
            self.diagnostic_callback, queue_size=1
        )

    def pose_callback(self, message):
        self.pose = message
        self.publish()

    def diagnostic_callback(self, message):
        if message.status:
            self.state = message.status[0].message
        self.publish()

    def publish(self):
        if self.pose is None:
            return
        color = COLORS.get(self.state, (0.6, 0.6, 0.6))
        marker = Marker()
        marker.header = self.pose.header
        marker.ns = "localization_state"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = self.pose.pose.pose
        marker.scale.x = 1.2
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        marker.text = self.state
        self.publisher.publish(marker)

        footprint = Marker()
        footprint.header = self.pose.header
        footprint.ns = "wheelchair_footprint"
        footprint.id = 0
        footprint.type = Marker.CYLINDER
        footprint.action = Marker.ADD
        footprint.pose = self.pose.pose.pose
        footprint.scale.x = footprint.scale.y = 1.0
        footprint.scale.z = 0.08
        footprint.color.r, footprint.color.g, footprint.color.b = color
        footprint.color.a = 0.45
        self.footprint_publisher.publish(footprint)


if __name__ == "__main__":
    rospy.init_node("localization_state_marker")
    LocalizationStateMarker()
    rospy.spin()

