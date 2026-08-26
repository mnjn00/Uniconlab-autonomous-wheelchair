#!/usr/bin/env python3
"""Fail-closed readiness check for the hybrid perception/avoidance profile."""

import json
import math
import sys

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String


class PreflightFailure(RuntimeError):
    pass


def payload(message, name):
    try:
        value = json.loads(message.data)
    except (TypeError, ValueError) as error:
        raise PreflightFailure("%s is not JSON: %s" % (name, error))
    if not isinstance(value, dict):
        raise PreflightFailure("%s is not an object" % name)
    return value


def wait(topic, message_type, timeout_s):
    try:
        return rospy.wait_for_message(topic, message_type, timeout=timeout_s)
    except rospy.ROSException:
        raise PreflightFailure("%s is silent" % topic)


def main():
    rospy.init_node("hybrid_preflight", anonymous=True, disable_signals=True)
    timeout_s = float(rospy.get_param("~timeout_s", 5.0))
    require_learned = bool(rospy.get_param("~require_learned", False))

    law = str(rospy.get_param("/waypoint_follower/control_law", ""))
    if law != "dwa":
        raise PreflightFailure(
            "running control law is %r, expected 'dwa'" % law)

    summary = payload(wait(
        "/perception/objects_summary", String, timeout_s),
        "objects_summary")
    if summary.get("status") != "OK":
        raise PreflightFailure(
            "fused perception status is %r" % summary.get("status"))
    if summary.get("frame") != "chair_centre":
        raise PreflightFailure(
            "fused perception frame is %r" % summary.get("frame"))
    stamp = summary.get("stamp")
    if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        raise PreflightFailure("fused perception has no finite stamp")
    age = rospy.Time.now().to_sec() - float(stamp)
    if age < -0.05 or age > 1.5:
        raise PreflightFailure("fused perception age is %.2f s" % age)
    if require_learned and summary.get("mode") != "hybrid":
        raise PreflightFailure(
            "learned perception was required but fusion mode is %r"
            % summary.get("mode"))

    hybrid = payload(wait(
        "/perception/hybrid_status", String, timeout_s),
        "hybrid_status")
    if hybrid.get("status") != "OK":
        raise PreflightFailure(
            "hybrid fusion is not ready: %r" % hybrid.get("status"))

    semantic = payload(wait(
        "/semantic_safety/status", String, timeout_s),
        "semantic_safety")
    if semantic.get("blocked"):
        raise PreflightFailure(
            "semantic supervisor is holding: %s"
            % semantic.get("reason"))

    terrain = payload(wait(
        "/terrain_guard/status", String, timeout_s),
        "terrain_guard")
    if terrain.get("blocked"):
        raise PreflightFailure(
            "terrain guard is holding: %s" % terrain.get("reason"))

    diagnostics = wait(
        "/fast_lio_icp/localization_diagnostics",
        DiagnosticArray, timeout_s)
    tracking = any(
        status.name == "fast_lio_icp" and status.message == "TRACKING"
        for status in diagnostics.status)
    if not tracking:
        raise PreflightFailure("localization is not TRACKING")

    print("HYBRID_PREFLIGHT_OK")
    print("  control law : dwa")
    print("  perception  : %s (%d objects)" % (
        summary.get("mode"), len(summary.get("objects", []))))
    print("  semantic    : ready")
    print("  terrain     : ready")
    print("  localization: TRACKING")


if __name__ == "__main__":
    try:
        main()
    except PreflightFailure as error:
        sys.stderr.write("HYBRID_PREFLIGHT_FAILED: %s\n" % error)
        raise SystemExit(1)
