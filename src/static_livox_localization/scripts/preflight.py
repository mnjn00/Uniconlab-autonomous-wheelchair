#!/usr/bin/env python3
"""Fail closed before starting the no-motion static localization trial."""

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


def evaluate(publishers, map_path, expected_hash):
    missing = [topic for topic in REQUIRED_TOPICS if not publishers.get(topic)]
    forbidden = sorted(topic for topic, nodes in publishers.items()
                       if nodes and topic.startswith(FORBIDDEN_PREFIXES))
    if missing:
        return 10, "missing publishers: " + ", ".join(missing)
    if not os.path.isfile(map_path) or sha256_file(map_path) != expected_hash:
        return 11, "map missing or SHA-256 mismatch"
    if forbidden:
        return 12, "motion command publishers active: " + ", ".join(forbidden)
    return 0, "preflight passed"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True)
    parser.add_argument("--sha256", required=True)
    args = parser.parse_args(argv)
    import rosgraph
    master = rosgraph.Master("/static_localization_preflight")
    publishers, _, _ = master.getSystemState()
    code, message = evaluate(dict(publishers), args.map, args.sha256)
    print(message)
    return code


if __name__ == "__main__":
    sys.exit(main())
