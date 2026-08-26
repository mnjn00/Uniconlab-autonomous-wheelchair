#!/usr/bin/env python3
"""Exit after observing the first committed advisory human state."""

import json

import rospy
from std_msgs.msg import String


def on_status(message):
    try:
        status = json.loads(message.data)
    except (TypeError, ValueError):
        return
    if status.get("decision") != "BYPASS_COMMITTED":
        return
    print(json.dumps(status, sort_keys=True), flush=True)
    rospy.signal_shutdown("first committed shadow state observed")


def main():
    rospy.init_node("wait_for_cohan_shadow_commit")
    rospy.Subscriber(
        "/human_aware_shadow/status",
        String,
        on_status,
        queue_size=10,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
