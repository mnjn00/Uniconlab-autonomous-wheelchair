#!/usr/bin/env python3
import json
import os
import sys

import rospy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from person_bypass_policy import PERMIT_SCHEMA, permit_from_payload, permit_is_fresh


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

    permit_data, permit_raw = wait_json(
        "/static_threat_bypass/permit", timeout_s)
    permit = permit_from_payload(permit_raw)
    if permit_data.get("schema") != PERMIT_SCHEMA:
        raise Failure("static-threat permit is not strict v2")
    if permit is None or not permit.capable:
        raise Failure("static-threat permit publisher is not capable")
    if not permit_is_fresh(
            permit, rospy.Time.now().to_sec(), maximum_permit_age_s):
        raise Failure("static-threat permit heartbeat is stale")

    semantic, _ = wait_json("/semantic_safety/status", timeout_s)
    if semantic.get("static_threat_bypass_capable") is not True:
        raise Failure("semantic supervisor is the stop-only implementation")

    gate, _ = wait_json("/safety_gate/status", timeout_s)
    if gate.get("static_threat_bypass_capable") is not True:
        raise Failure("raw safety gate is the fixed-corridor implementation")
    if gate.get("static_threat_bypass_proposal_capable") is not True:
        raise Failure("raw safety gate cannot validate trajectory proposals")

    parameters = (
        ("/waypoint_follower/static_threat_bypass_capable", True),
        ("/waypoint_follower/static_threat_bypass_proposal_capable", True),
        ("/semantic_safety_supervisor/static_threat_bypass_capable", True),
        ("/safety_gate/static_threat_bypass_capable", True),
        ("/safety_gate/static_threat_bypass_proposal_capable", True),
    )
    for name, expected in parameters:
        if rospy.get_param(name, None) is not expected:
            raise Failure("%s does not prove the bypass implementation" % name)

    confirmation_name = (
        "/waypoint_follower/static_threat_bypass_confirmation_s")
    confirmation_s = rospy.get_param(confirmation_name, None)
    if isinstance(confirmation_s, bool) or not isinstance(
            confirmation_s, (int, float)) or float(confirmation_s) != 2.0:
        raise Failure("%s must be exactly 2.0" % confirmation_name)

    print("STATIC_THREAT_BYPASS_PREFLIGHT_OK")
    print("  qualifier : person or object, same-track STATIC for 2.0 s")
    print("  semantic  : one qualified static-threat exception")
    print("  raw gate  : exact trajectory-proposal validation")
    print("  permit    : %s" % permit_data.get("reason"))


if __name__ == "__main__":
    try:
        main()
    except Failure as error:
        sys.stderr.write("STATIC_THREAT_BYPASS_PREFLIGHT_FAILED: %s\n" % error)
        raise SystemExit(1)
