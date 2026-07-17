#!/usr/bin/env python3
"""Apply fixed acceptance limits to a moving localization JSON report."""

import argparse
import json
from pathlib import Path


def evaluate_summary(
    summary,
    max_pose_jump_m=0.50,
    max_yaw_jump_deg=10.0,
    max_return_translation_m=0.50,
    max_return_yaw_deg=10.0,
):
    reasons = []
    if not summary.get("pose_samples"):
        reasons.append("NO_POSE")
        return False, reasons
    if summary.get("max_position_step_m") is None or (
        summary["max_position_step_m"] > max_pose_jump_m
    ):
        reasons.append("POSE_JUMP")
    if summary.get("max_yaw_step_deg") is None or (
        summary["max_yaw_step_deg"] > max_yaw_jump_deg
    ):
        reasons.append("YAW_JUMP")
    if summary.get("return_translation_m") is None or (
        summary["return_translation_m"] > max_return_translation_m
    ):
        reasons.append("RETURN_TRANSLATION")
    if summary.get("return_yaw_deg") is None or (
        summary["return_yaw_deg"] > max_return_yaw_deg
    ):
        reasons.append("RETURN_YAW")
    if summary.get("state_duration_s", {}).get("LOST", 0.0) > 0.0:
        reasons.append("LOST_STATE")
    return not reasons, reasons


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report")
    args = parser.parse_args(argv)
    summary = json.loads(Path(args.report).read_text(encoding="utf-8"))
    accepted, reasons = evaluate_summary(summary)
    print("PASS" if accepted else "FAIL: " + ", ".join(reasons))
    return 0 if accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
