#!/usr/bin/env python3
"""Fail closed before starting fixed-map Livox localization."""

import argparse
import hashlib
import os
import sys


REQUIRED_TOPICS = ("/livox/lidar", "/livox/imu", "/cloud_registered_body", "/Odometry")
FORBIDDEN_PREFIXES = ("/cmd_vel",)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def evaluate(publishers, map_path, expected_hash, topic_ages=None,
             max_topic_age_s=1.0, tf_authorities=None):
    missing = [topic for topic in REQUIRED_TOPICS if not publishers.get(topic)]
    duplicates = sorted(topic for topic in REQUIRED_TOPICS
                        if len(publishers.get(topic, ())) > 1)
    forbidden = sorted(topic for topic, nodes in publishers.items()
                       if nodes and topic.startswith(FORBIDDEN_PREFIXES))
    stale = []
    if topic_ages is not None:
        stale = sorted(topic for topic in REQUIRED_TOPICS
                       if topic_ages.get(topic, float("inf")) > max_topic_age_s)
    if missing:
        return 10, "missing publishers: " + ", ".join(missing)
    if duplicates:
        return 10, "duplicate publishers: " + ", ".join(duplicates)
    if stale:
        return 10, "stale required topics: " + ", ".join(stale)
    if not os.path.isfile(map_path) or sha256_file(map_path) != expected_hash:
        return 11, "map missing or SHA-256 mismatch"
    if forbidden:
        return 12, "motion command publishers active: " + ", ".join(forbidden)
    if tf_authorities:
        return 13, "map to odom TF authority already active: " + ", ".join(
            sorted(set(tf_authorities)))
    return 0, "preflight passed"


def collect_topic_ages(timeout_s):
    import rospy
    import rostopic
    import time

    ages = {}
    for topic in REQUIRED_TOPICS:
        message_class, _, _ = rostopic.get_topic_class(topic, blocking=False)
        if message_class is None:
            ages[topic] = float("inf")
            continue
        started = time.monotonic()
        try:
            rospy.wait_for_message(topic, message_class, timeout=timeout_s)
        except rospy.ROSException:
            ages[topic] = float("inf")
        else:
            ages[topic] = time.monotonic() - started
    return ages


def collect_tf_authorities(map_frame, odom_frame, observe_s):
    import rospy
    from tf2_msgs.msg import TFMessage

    authorities = set()

    def callback(message):
        caller = getattr(message, "_connection_header", {}).get(
            "callerid", "unknown")
        for transform in message.transforms:
            if (transform.header.frame_id == map_frame and
                    transform.child_frame_id == odom_frame):
                authorities.add(caller)

    subscriber = rospy.Subscriber("/tf", TFMessage, callback, queue_size=100)
    rospy.sleep(observe_s)
    subscriber.unregister()
    return sorted(authorities)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True)
    parser.add_argument("--sha256", required=True)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="camera_init")
    parser.add_argument("--topic-timeout-s", type=float, default=1.0)
    parser.add_argument("--tf-observe-s", type=float, default=1.0)
    args = parser.parse_args(argv)
    import rosgraph
    import rospy

    rospy.init_node("moving_localization_preflight", anonymous=True,
                    disable_signals=True)
    master = rosgraph.Master("/moving_localization_preflight")
    publishers, _, _ = master.getSystemState()
    topic_ages = collect_topic_ages(args.topic_timeout_s)
    tf_authorities = collect_tf_authorities(
        args.map_frame, args.odom_frame, args.tf_observe_s)
    code, message = evaluate(
        dict(publishers), args.map, args.sha256,
        topic_ages=topic_ages, max_topic_age_s=args.topic_timeout_s,
        tf_authorities=tf_authorities)
    print(message)
    return code


if __name__ == "__main__":
    sys.exit(main())
