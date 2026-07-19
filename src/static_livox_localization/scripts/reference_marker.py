#!/usr/bin/env python3
"""Publish a visual-only marker for an operator-selected checkpoint."""

import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker


class ReferenceMarker:
    def __init__(self):
        self.publisher = rospy.Publisher(
            "/fast_lio_icp/reference_marker", Marker, queue_size=1, latch=True
        )
        self.subscriber = rospy.Subscriber(
            "/clicked_point", PointStamped, self.callback, queue_size=1
        )

    def callback(self, point):
        marker = Marker()
        marker.header = point.header
        marker.ns = "operator_reference"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.45
        marker.color.r = 0.0
        marker.color.g = 0.35
        marker.color.b = 1.0
        marker.color.a = 0.95
        self.publisher.publish(marker)


if __name__ == "__main__":
    rospy.init_node("reference_marker")
    ReferenceMarker()
    rospy.spin()

