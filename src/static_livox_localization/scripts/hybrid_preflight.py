#!/usr/bin/env python3
"""Fail-closed readiness check for the RTX hybrid perception/avoidance graph."""

import json
import math
import sys

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String


PINNED_POINTPILLARS_COMMIT = \
    "ce7e2bd694c90207435c8751d61cdb38d48a9f4c"


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


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or \
            not math.isfinite(float(value)):
        raise PreflightFailure("%s is not finite" % name)
    return float(value)


def fresh_stamp(data, name, maximum_age_s):
    stamp = finite(data.get("stamp"), "%s stamp" % name)
    age = rospy.Time.now().to_sec() - stamp
    if age < -0.05 or age > maximum_age_s:
        raise PreflightFailure("%s age is %.2f s" % (name, age))
    return age


def route_contract():
    route_path = str(rospy.get_param("/waypoint_follower/route", ""))
    if not route_path:
        raise PreflightFailure("running follower publishes no route path")
    try:
        with open(route_path, encoding="utf-8") as stream:
            route = json.load(stream)
        profile = str(route["body_frame_profile"])
        centre = [float(value) for value in
                  route["chair_centre_in_body_xyz"]]
    except (IOError, OSError, KeyError, TypeError, ValueError) as error:
        raise PreflightFailure(
            "cannot read running route frame contract: %s" % error)
    if len(centre) != 3 or not all(math.isfinite(value) for value in centre):
        raise PreflightFailure("running route chair centre is invalid")
    return route_path, profile, centre


def require_dwa_gpu():
    active = rospy.get_param("/waypoint_follower/gpu_active", None)
    backend = str(rospy.get_param(
        "/waypoint_follower/distance_backend", "")).strip().lower()
    if active is not True or backend != "cupy":
        raise PreflightFailure(
            "DWA is not using RTX/CuPy: gpu_active=%r backend=%r"
            % (active, backend))
    return backend


def require_pointpillars(timeout_s, require_rtx2060,
                         maximum_age_s, maximum_inference_ms):
    gpu = payload(wait(
        "/pointpillars/status", String, timeout_s),
        "pointpillars_status")
    if gpu.get("status") != "OK":
        raise PreflightFailure(
            "PointPillars status is %r: %s" %
            (gpu.get("status"), gpu.get("detail", "")))
    if gpu.get("gpu_active") is not True:
        raise PreflightFailure("PointPillars did not report GPU execution")
    device_name = str(gpu.get("device_name", ""))
    if require_rtx2060 and "RTX 2060" not in device_name:
        raise PreflightFailure(
            "PointPillars is running on %r, not RTX 2060" % device_name)
    if gpu.get("upstream_commit") != PINNED_POINTPILLARS_COMMIT:
        raise PreflightFailure(
            "PointPillars upstream commit is not the reviewed pin")
    fresh_stamp(gpu, "PointPillars", maximum_age_s)
    inference_ms = finite(
        gpu.get("inference_ms"), "PointPillars inference_ms")
    if inference_ms > maximum_inference_ms:
        raise PreflightFailure(
            "PointPillars inference %.1f ms exceeds %.1f ms" %
            (inference_ms, maximum_inference_ms))
    try:
        used_points = int(gpu.get("used_points", 0))
    except (TypeError, ValueError):
        used_points = 0
    if used_points <= 0:
        raise PreflightFailure("PointPillars processed no LiDAR points")
    return gpu


def main():
    rospy.init_node("hybrid_preflight", anonymous=True, disable_signals=True)
    timeout_s = float(rospy.get_param("~timeout_s", 5.0))
    require_learned = bool(rospy.get_param("~require_learned", False))
    require_gpu_detector = bool(rospy.get_param(
        "~require_gpu_detector", require_learned))
    require_rtx2060 = bool(rospy.get_param("~require_rtx2060", True))
    require_gpu_dwa = bool(rospy.get_param("~require_gpu_dwa", True))
    maximum_gpu_age_s = float(rospy.get_param(
        "~maximum_gpu_status_age_s", 1.5))
    maximum_inference_ms = float(rospy.get_param(
        "~maximum_gpu_inference_ms", 90.0))

    law = str(rospy.get_param("/waypoint_follower/control_law", ""))
    if law != "dwa":
        raise PreflightFailure(
            "running control law is %r, expected 'dwa'" % law)
    route_path, expected_profile, expected_centre = route_contract()
    dwa_backend = require_dwa_gpu() if require_gpu_dwa else "not-required"

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
    fresh_stamp(summary, "fused perception", 1.5)
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

    pointpillars = None
    if require_gpu_detector:
        pointpillars = require_pointpillars(
            timeout_s, require_rtx2060, maximum_gpu_age_s,
            maximum_inference_ms)

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
    print("  DWA backend : %s" % dwa_backend)
    print("  route       : %s" % route_path)
    print("  object frame: chair_centre/%s" % expected_profile)
    print("  perception  : %s (%d objects)" % (
        summary.get("mode"), len(summary.get("objects", []))))
    if pointpillars is not None:
        print("  detector GPU: %s" % pointpillars.get("device_name"))
        print("  inference    : %.2f ms" %
              float(pointpillars["inference_ms"]))
        print("  GPU points   : %s" % pointpillars.get("used_points"))
    else:
        print("  detector GPU: not required")
    print("  semantic    : ready")
    print("  terrain     : ready")
    print("  localization: TRACKING")


if __name__ == "__main__":
    try:
        main()
    except PreflightFailure as error:
        sys.stderr.write("HYBRID_PREFLIGHT_FAILED: %s\n" % error)
        raise SystemExit(1)
