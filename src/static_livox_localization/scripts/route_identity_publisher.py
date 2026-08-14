#!/usr/bin/env python3
"""Publish the immutable route asset identity into every black-box bag."""

import json
import os
import sys

import rospy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from route_assets import validate_asset_binding


def main():
    rospy.init_node("route_identity")
    binding = validate_asset_binding(
        rospy.get_param("~route"),
        rospy.get_param("~safety_band"),
        rospy.get_param("~drivable_mask"),
    )
    publisher = rospy.Publisher(
        "/waypoint_follower/route_identity",
        String,
        queue_size=1,
        latch=True,
    )
    publisher.publish(String(data=json.dumps(binding, sort_keys=True)))
    rospy.spin()


if __name__ == "__main__":
    main()
