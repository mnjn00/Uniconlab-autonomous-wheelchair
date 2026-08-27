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


def route_contract():
    route_path = str(rospy.get_param("/waypoint_follower/route", ""))
    if not route_path:
        raise PreflightFailure("running follower publishes no route path")
    try:
        with open(route_path, encoding="utf-8") as stream:
            route = json.load(stream)
        profile = str(route["body_frame_profile"])
        centre = route["chair_centre_in_body_xyz"]
        centre = [float(value) for value in centre]
    except (IOError, OSError, KeyError, TypeError, ValueError) as error:
        raise PreflightFailure(
            "cannot read running route frame contract: %s" % error)
    if len(centre) != 3 or not all(math.isfinite(value) for value in centre):
        raise PreflightFailure("running route chair centre is invalid")
    return route_path, profile, centre


def main():
    rospy.init_node("hybrid_preflight", anonymous=True, disable_signals=True)
    timeout_s = float(rospy.get_param("~timeout_s", 5.0))
    require_learned = bool(rospy.get_param("~require_learned", False))

    law = str(rospy.get_param("/waypoint_follower/control_law", ""))
    if law != "dwa":
        raise PreflightFailure(
            "running control law is %r, expected 'dwa'" % law)
    route_path, expected_profile, expected_centre = route_contract()

    summary = payload(wait(
        "/perception/objects_summary", String, timeout_s),
        "objects_summary")
    if summary.get("status") != "OK":
        raise PreflightFailure(
            "fused perception status is %r" % summary.get("status"))
    if summary.get("frame") != "chair_centre":
        raise PreflightFailure(
            "fused perception frame is %r" % summary.get("frame"))
    if summary.get("body_frame_profile") != expected_profile:
        raise PreflightFailure(
            "fused object profile %r does not match route profile %r"
            % (summary.get("body_frame_profile"), expected_profile))
    observed_centre = summary.get("chair_centre_in_body_xyz")
    try:
        observed_centre = [float(value) for value in observed_centre]
    except (TypeError, ValueError):
        raise PreflightFailure("fused summary has no chair-centre contract")
    if len(observed_centre) != 3 or any(
            abs(observed - expected) > 1e-6
            for observed, expected in zip(observed_centre, expected_centre)):
        raise PreflightFailure(
            "fused chair centre %r does not match route %r"
            % (observed_centre, expected_centre))

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
    if hybrid.get("output_body_frame_profile") != expected_profile:
        raise PreflightFailure(
            "hybrid health profile does not match the route")

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
    print("  route       : %s" % route_path)
    print("  object frame: chair_centre/%s" % expected_profile)
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