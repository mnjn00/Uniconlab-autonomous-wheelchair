#!/usr/bin/env python3
"""Fail when any ROS motion-control surface is connected during shadow QA."""

import json
import os
import argparse
import sys
import xmlrpc.client
from pathlib import Path

SCRIPTS = (
    Path(__file__).parents[1]
    / "src"
    / "static_livox_localization"
    / "scripts"
)
sys.path.insert(0, str(SCRIPTS))
try:
    from shadow_qa_contract import (
        baseline_node_violations,
        motion_surface_violations,
    )
finally:
    sys.path.pop(0)


def system_state(master_uri=None):
    """Read one atomic ROS-master graph snapshot or fail closed."""
    uri = master_uri or os.environ.get("ROS_MASTER_URI")
    if not uri:
        raise RuntimeError("ROS_MASTER_URI is not set")
    code, message, state = xmlrpc.client.ServerProxy(uri).getSystemState(
        "/shadow_qa_audit"
    )
    if code != 1:
        raise RuntimeError("ROS master graph query failed: %s" % message)
    publishers, subscribers, _services = state
    return dict(publishers), dict(subscribers)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline")
    args = parser.parse_args()
    try:
        publishers, subscribers = system_state()
    except Exception as error:
        print(json.dumps({"status": "ERROR", "error": str(error)}))
        return 2
    violations = motion_surface_violations(publishers, subscribers)
    if violations:
        print(json.dumps({
            "status": "UNSAFE",
            "violations": violations,
        }))
        return 1
    nodes = sorted({
        node
        for graph in (publishers, subscribers)
        for connected in graph.values()
        for node in connected
    })
    if args.baseline:
        baseline = json.loads(Path(args.baseline).read_text(encoding="utf-8"))
        baseline_violations = baseline_node_violations(nodes, baseline)
        if baseline_violations:
            print(json.dumps({
                "status": "UNSAFE",
                "violations": baseline_violations,
                "nodes": nodes,
            }))
            return 1
    print(json.dumps({"status": "SAFE", "violations": [], "nodes": nodes}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
