#!/usr/bin/env python3
"""Fail-closed readiness check for the stationary-person bypass branch."""

import json
import sys

import rospy
from std_msgs.msg import String

from person_bypass_policy import permit_from_payload, permit_is_fresh


class Failure(RuntimeError):
    pass


def wait_json(topic, timeout_s):
    try:
        message = rospy.wait_for_message(topic, String, timeout=timeout_s)
    except rospy.ROSException:
        raise Failure("%s is silent" % topic)
    try:
        value = json.loads(message.data)
    except (TypeError, ValueError) as error:
        raise Failure("%s is not JSON: %s" % (topic, error))
    if not isinstance(value, dict):
        raise Failure("%s is not an object" % topic)
    return value, message.data


def main():
    rospy.init_node(
        "person_bypass_preflight", anonymous=True, disable_signals=True)
    timeout_s = float(rospy.get_param("~timeout_s", 5.0))
    maximum_permit_age_s = float(rospy.get_param(
        "~maximum_permit_age_s", 0.60))

    permit_data, permit_raw = wait_json("/person_bypass/permit", timeout_s)
    permit = permit_from_payload(permit_raw)
    if permit is None or not permit.capable:
        raise Failure("person-bypass permit publisher is not capable")
    if not permit_is_fresh(
            permit, rospy.Time.now().to_sec(), maximum_permit_age_s):
        raise Failure("person-bypass permit heartbeat is stale")

    semantic, _ = wait_json("/semantic_safety/status", timeout_s)
    if semantic.get("person_bypass_capable") is not True:
        raise Failure("semantic supervisor is the stop-only implementation")

    gate, _ = wait_json("/safety_gate/status", timeout_s)
    if gate.get("trajectory_person_bypass_capable") is not True:
        raise Failure("raw safety gate is the fixed-corridor implementation")

    parameters = (
        ("/waypoint_follower/person_bypass_capable", True),
        ("/semantic_safety_supervisor/person_bypass_capable", True),
        ("/safety_gate/trajectory_person_bypass_capable", True),
    )
    for name, expected in parameters:
        if rospy.get_param(name, None) is not expected:
            raise Failure("%s does not prove the bypass implementation" % name)

    print("PERSON_BYPASS_PREFLIGHT_OK")
    print("  qualifier : continuous same-track STATIC")
    print("  semantic  : target-only static-person exception")
    print("  raw gate  : curved swept-footprint validation")
    print("  permit    : %s" % permit_data.get("reason"))


if __name__ == "__main__":
    try:
        main()
    except Failure as error:
        sys.stderr.write("PERSON_BYPASS_PREFLIGHT_FAILED: %s\n" % error)
        raise SystemExit(1)
