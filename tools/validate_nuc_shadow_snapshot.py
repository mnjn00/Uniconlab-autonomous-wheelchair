#!/usr/bin/env python3
"""Validate ROS YAML captured by run_nuc_shadow_qa.sh."""

import argparse
import json
import sys
from pathlib import Path

import yaml

SCRIPTS = (
    Path(__file__).parents[1]
    / "src"
    / "static_livox_localization"
    / "scripts"
)
sys.path.insert(0, str(SCRIPTS))
try:
    from shadow_qa_contract import validate_snapshot
finally:
    sys.path.pop(0)


def load_yaml(path):
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("%s is not a ROS message object" % path)
    return document


def parse_summary(document):
    payload = document.get("data")
    if not isinstance(payload, str):
        raise ValueError("objects_summary has no String.data")
    value = json.loads(payload)
    if not isinstance(value, dict):
        raise ValueError("objects_summary data is not an object")
    return value


def parse_boxes(document):
    boxes = []
    for marker in document.get("markers", []):
        if marker.get("action") != 0:
            continue
        header = marker.get("header", {})
        if header.get("frame_id") != "body":
            raise ValueError("dynamic box is not in body frame")
        scale = marker.get("scale", {})
        position = marker.get("pose", {}).get("position", {})
        stamp = header.get("stamp", {})
        boxes.append({
            "id": marker.get("id"),
            "size": [scale.get("x"), scale.get("y"), scale.get("z")],
            "position": [
                position.get("x"), position.get("y"), position.get("z")
            ],
            "stamp": (
                float(stamp.get("secs", 0))
                + float(stamp.get("nsecs", 0)) * 1e-9
            ),
        })
    return boxes


def parse_diagnostics(document):
    statuses = document.get("status", [])
    if not statuses:
        raise ValueError("localization_diagnostics has no status")
    selected = next(
        (
            status
            for status in statuses
            if "localization" in str(status.get("name", "")).lower()
        ),
        statuses[0],
    )
    values = selected.get("values", [])
    parsed = {
        str(item.get("key")): str(item.get("value"))
        for item in values
        if item.get("key") is not None
    }
    if "tracking_state" not in parsed and "raw_state" in parsed:
        parsed["tracking_state"] = parsed["raw_state"]
    header = document.get("header", {})
    stamp = header.get("stamp", {})
    parsed["_stamp"] = (
        float(stamp.get("secs", 0))
        + float(stamp.get("nsecs", 0)) * 1e-9
    )
    return parsed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", required=True)
    parser.add_argument("--boxes", required=True)
    parser.add_argument("--diagnostics", required=True)
    args = parser.parse_args()

    summary = parse_summary(load_yaml(args.summary))
    boxes = parse_boxes(load_yaml(args.boxes))
    diagnostics = parse_diagnostics(load_yaml(args.diagnostics))
    diagnostics_stamp = diagnostics.pop("_stamp", None)
    evidence = validate_snapshot(
        summary, boxes, diagnostics,
        diagnostics_stamp=diagnostics_stamp,
    )
    print(json.dumps({"status": "PASS", **evidence}, sort_keys=True))


if __name__ == "__main__":
    main()
