#!/usr/bin/env python3
"""Summarize moving localization pose and state evidence from a rosbag."""

import argparse
import json
import math
from pathlib import Path


POSE_TOPIC = "/fast_lio_icp/pose"
DIAGNOSTIC_TOPIC = "/fast_lio_icp/localization_diagnostics"


def yaw_from_quaternion(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def wrapped_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def summarize(poses, states):
    poses = sorted(poses, key=lambda item: item["stamp"])
    states = sorted(states, key=lambda item: item["stamp"])
    result = {
        "pose_samples": len(poses),
        "state_samples": len(states),
        "initialization_time_s": None,
        "state_duration_s": {},
        "max_position_step_m": None,
        "max_yaw_step_deg": None,
        "return_translation_m": None,
        "return_yaw_deg": None,
    }

    end_candidates = []
    if poses:
        end_candidates.append(poses[-1]["stamp"])
    if states:
        end_candidates.append(states[-1]["stamp"])
    end_stamp = max(end_candidates) if end_candidates else 0.0
    for index, sample in enumerate(states):
        next_stamp = (
            states[index + 1]["stamp"] if index + 1 < len(states) else end_stamp
        )
        duration = max(0.0, next_stamp - sample["stamp"])
        name = sample["state"]
        result["state_duration_s"][name] = (
            result["state_duration_s"].get(name, 0.0) + duration
        )

    if not poses:
        return result

    trial_start = states[0]["stamp"] if states else poses[0]["stamp"]
    result["initialization_time_s"] = max(0.0, poses[0]["stamp"] - trial_start)
    yaws = [
        yaw_from_quaternion(
            sample["qx"], sample["qy"], sample["qz"], sample["qw"]
        )
        for sample in poses
    ]
    position_steps = []
    yaw_steps = []
    for previous, current, previous_yaw, current_yaw in zip(
        poses, poses[1:], yaws, yaws[1:]
    ):
        position_steps.append(
            math.sqrt(
                (current["x"] - previous["x"]) ** 2
                + (current["y"] - previous["y"]) ** 2
                + (current["z"] - previous["z"]) ** 2
            )
        )
        yaw_steps.append(abs(wrapped_angle(current_yaw - previous_yaw)))
    result["max_position_step_m"] = max(position_steps, default=0.0)
    result["max_yaw_step_deg"] = math.degrees(max(yaw_steps, default=0.0))

    first = poses[0]
    last = poses[-1]
    result["return_translation_m"] = math.sqrt(
        (last["x"] - first["x"]) ** 2
        + (last["y"] - first["y"]) ** 2
        + (last["z"] - first["z"]) ** 2
    )
    result["return_yaw_deg"] = math.degrees(
        abs(wrapped_angle(yaws[-1] - yaws[0]))
    )
    return result


def extract_bag(path):
    import rosbag

    poses = []
    states = []
    with rosbag.Bag(path, "r") as bag:
        for topic, message, bag_stamp in bag.read_messages(
            topics=(POSE_TOPIC, DIAGNOSTIC_TOPIC)
        ):
            header_stamp = getattr(getattr(message, "header", None), "stamp", None)
            stamp = (
                header_stamp.to_sec()
                if header_stamp is not None and not header_stamp.is_zero()
                else bag_stamp.to_sec()
            )
            if topic == POSE_TOPIC:
                position = message.pose.pose.position
                orientation = message.pose.pose.orientation
                poses.append(
                    {
                        "stamp": stamp,
                        "x": position.x,
                        "y": position.y,
                        "z": position.z,
                        "qx": orientation.x,
                        "qy": orientation.y,
                        "qz": orientation.z,
                        "qw": orientation.w,
                    }
                )
            elif message.status:
                status = message.status[0]
                values = {item.key: item.value for item in status.values}
                states.append(
                    {
                        "stamp": stamp,
                        "state": status.message,
                        "reason": values.get("reason", ""),
                        "fitness": values.get("fitness"),
                        "inlier_ratio": values.get("inlier_ratio"),
                    }
                )
    return poses, states


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    poses, states = extract_bag(args.bag)
    report = summarize(poses, states)
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0 if poses else 2


if __name__ == "__main__":
    raise SystemExit(main())

