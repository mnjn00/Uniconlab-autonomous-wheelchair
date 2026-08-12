#!/usr/bin/env python3
"""Fail when any ROS motion-control surface is connected during shadow QA."""

import json
import subprocess
import sys
from pathlib import Path

SCRIPTS = (
    Path(__file__).parents[1]
    / "src"
    / "static_livox_localization"
    / "scripts"
)
sys.path.insert(0, str(SCRIPTS))
try:
    from shadow_qa_contract import COMMAND_TOPICS, motion_surface_violations
finally:
    sys.path.pop(0)


def topic_nodes(topic):
    output = subprocess.run(
        ["rostopic", "info", topic],
        check=False,
        capture_output=True,
        text=True,
    )
    if output.returncode != 0:
        return [], []
    publishers, subscribers = [], []
    target = None
    for line in output.stdout.splitlines():
        stripped = line.strip()
        if stripped == "Publishers:":
            target = publishers
        elif stripped == "Subscribers:":
            target = subscribers
        elif stripped.startswith("* /") and target is not None:
            target.append(stripped.split()[1])
    return publishers, subscribers


def main():
    publishers, subscribers = {}, {}
    for topic in COMMAND_TOPICS:
        topic_publishers, topic_subscribers = topic_nodes(topic)
        publishers[topic] = topic_publishers
        subscribers[topic] = topic_subscribers
    violations = motion_surface_violations(publishers, subscribers)
    if violations:
        print(json.dumps({"status": "UNSAFE", "violations": violations}))
        return 1
    print(json.dumps({"status": "SAFE", "violations": []}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
