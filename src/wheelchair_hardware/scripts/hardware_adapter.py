#!/usr/bin/env python3
"""Fail-closed ROS adapter for the measured wheelchair driver contract."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
_SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, _SCRIPT_DIRECTORY)

_CANONICAL_DRIVER_TOPIC = "/hardware/driver_status"
_CANONICAL_MODE_TOPIC = "/safety/mode"
_CANONICAL_DRIVER_SIGNAL_TOPIC = "/safety/driver"
_STATUS_PERIOD_S = 0.05

_ESTOP_REASON = 1 << 0
_MODE_REASON = 1 << 2
_DRIVER_REASON = 1 << 6
_SENSOR_STALE_REASON = 1 << 12
_MANUAL_OVERRIDE_REASON = 1 << 24
_INPUT_UNKNOWN_REASON = 1 << 32
from driver_contract import DriverContractError, load_manifest, preflight


_REQUIRED_RUNTIME_EVIDENCE = ("platform_matches", "base_model_matches", "graph_valid")


def _bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _twist_is_finite(message):
    values = (
        message.linear.x,
        message.linear.y,
        message.linear.z,
        message.angular.x,
        message.angular.y,
        message.angular.z,
    )
    return all(math.isfinite(value) for value in values)


def _adapter_preflight(manifest, profile, runtime_evidence):
    """Evaluate the unchanged signed driver contract; authority is never rewritten."""
    return preflight(manifest, profile, runtime_evidence=runtime_evidence)


def _endpoint_authorized(
    manifest, authority, result, profile, hardware_enable, runtime_evidence
):
    endpoint_allowed = getattr(
        result,
        "real_motor_path",
        getattr(result, "deployable", getattr(result, "create_motor_endpoint", False)),
    )
    return (
        profile == "hardware_enabled"
        and hardware_enable
        and bool(manifest.get("verified"))
        and manifest.get("hardware_motion_authorized") is True
        and manifest.get("passenger_operation_authorized") is True
        and (authority.get("release_scope") or {}).get("hardware_motion_authorized") is True
        and (authority.get("release_scope") or {}).get("passenger_operation_authorized") is True
        and (authority.get("blocked_profiles") or {}).get("hardware_enabled", {}).get("allowed") is True
        and runtime_evidence.get("receipt_verified") is True
        and all(runtime_evidence.get(key) is True for key in _REQUIRED_RUNTIME_EVIDENCE)
        and bool(getattr(result, "allowed", False))
        and bool(endpoint_allowed)
    )


def _fresh(sample, now, timeout):
    return (
        sample["stamp"] is not None
        and math.isfinite(timeout)
        and timeout > 0.0
        and 0.0 <= (now - sample["stamp"]).to_sec() <= timeout
    )


def _active(value, configured):
    return str(value).strip().lower() == str(configured).strip().lower()


def _evidence_decision(manifest, enabled, samples, now):
    mode = manifest.get("mode") or {}
    override = manifest.get("manual_override") or {}
    estop = manifest.get("estop") or {}
    fresh = (
        _fresh(samples["mode"], now, float(mode.get("stale_timeout_s") or 0.0))
        and _fresh(samples["override"], now, float(override.get("stale_timeout_s") or 0.0))
        and _fresh(samples["estop"], now, float(estop.get("stale_timeout_s") or 0.0))
    )
    mode_auto = fresh and _active(samples["mode"]["value"], mode.get("auto_value"))
    override_active = fresh and _active(samples["override"]["value"], override.get("active_value"))
    estop_asserted = fresh and _active(samples["estop"]["value"], estop.get("asserted_value"))
    clear = bool(enabled and fresh and mode_auto and not override_active and not estop_asserted)
    if clear:
        reason = 0
    elif not enabled:
        reason = _DRIVER_REASON | _INPUT_UNKNOWN_REASON
    elif not fresh:
        reason = _DRIVER_REASON | _SENSOR_STALE_REASON | _INPUT_UNKNOWN_REASON
    elif estop_asserted:
        reason = _ESTOP_REASON | _DRIVER_REASON
    elif override_active:
        reason = _MANUAL_OVERRIDE_REASON | _DRIVER_REASON
    else:
        reason = _MODE_REASON | _DRIVER_REASON
    return clear, fresh, mode_auto, override_active, estop_asserted, reason


def _evidence_message_class(message_type):
    from std_msgs.msg import Bool, String

    classes = {
        "std_msgs/Bool": Bool,
        "std_msgs/msg/Bool": Bool,
        "std_msgs/String": String,
        "std_msgs/msg/String": String,
    }
    try:
        return classes[str(message_type)]
    except KeyError as exc:
        raise DriverContractError(
            "E_FORMAT", "unsupported runtime evidence type: {}".format(message_type)
        ) from exc


def _parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=("hardware_shadow", "hardware_enabled"), required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--release-authority", required=True)
    parser.add_argument("--hardware-enable", default="false")
    parser.add_argument("--bundle-root", default="")
    parser.add_argument("--runtime-evidence", default="")
    parser.add_argument("--runtime-evidence-sha256", default="")
    return parser.parse_known_args()[0]


def _contract_hash(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256(value):
    return isinstance(value, str) and len(value) == 64 and all(
        character in "0123456789abcdef" for character in value
    )


def _contained_regular_file(root, relative):
    root = Path(root)
    if root.is_symlink() or not root.is_dir():
        raise DriverContractError("E_FORMAT", "runtime bundle root is unsafe")
    value = Path(relative)
    if not relative or value.is_absolute() or ".." in value.parts:
        raise DriverContractError("E_FORMAT", "runtime evidence path is unsafe")
    candidate = root / value
    cursor = candidate
    while cursor != root:
        if cursor.is_symlink():
            raise DriverContractError("E_FORMAT", "runtime evidence path contains a symlink")
        cursor = cursor.parent
    if not candidate.is_file():
        raise DriverContractError("E_FORMAT", "runtime evidence file is missing")
    try:
        candidate.resolve(strict=True).relative_to(root.resolve(strict=True))
    except (OSError, ValueError) as exc:
        raise DriverContractError("E_FORMAT", "runtime evidence escapes bundle root") from exc
    return candidate


def _load_runtime_evidence(bundle_root, receipt_relative, receipt_sha256,
                           manifest_path, authority_path):
    """Verify measured runtime facts and every file identity before ROS exists."""
    if not _sha256(receipt_sha256):
        raise DriverContractError("E_FORMAT", "runtime evidence SHA-256 is invalid")
    root = Path(bundle_root)
    receipt_path = _contained_regular_file(root, receipt_relative)
    if _contract_hash(receipt_path) != receipt_sha256:
        raise DriverContractError("E_FORMAT", "runtime evidence SHA-256 mismatch")
    try:
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise DriverContractError("E_FORMAT", "runtime evidence is malformed") from exc
    required = {
        "schema_version", "status", "driver_manifest_path", "driver_manifest_sha256",
        "release_authority_path", "release_authority_sha256", "bundle_manifest_path",
        "bundle_manifest_sha256", "platform_matches", "base_model_matches", "graph_valid",
    }
    if not isinstance(receipt, dict) or set(receipt) != required:
        raise DriverContractError("E_FORMAT", "runtime evidence fields mismatch")
    if receipt.get("schema_version") != 1 or receipt.get("status") != "verified":
        raise DriverContractError("E_FORMAT", "runtime evidence is not verified")
    bindings = (
        ("driver_manifest", Path(manifest_path).resolve(strict=True)),
        ("release_authority", Path(authority_path).resolve(strict=True)),
        ("bundle_manifest", None),
    )
    for prefix, expected_path in bindings:
        relative = receipt.get(prefix + "_path")
        digest = receipt.get(prefix + "_sha256")
        if not isinstance(relative, str) or not _sha256(digest):
            raise DriverContractError("E_FORMAT", prefix + " binding is invalid")
        candidate = _contained_regular_file(root, relative)
        if expected_path is not None and candidate.resolve(strict=True) != expected_path:
            raise DriverContractError("E_FORMAT", prefix + " path mismatch")
        if _contract_hash(candidate) != digest:
            raise DriverContractError("E_FORMAT", prefix + " SHA-256 mismatch")
    evidence = {key: receipt.get(key) for key in _REQUIRED_RUNTIME_EVIDENCE}
    if any(type(value) is not bool or value is not True for value in evidence.values()):
        raise DriverContractError("E_FORMAT", "runtime evidence facts are not all measured true")
    evidence["receipt_verified"] = True
    return evidence


def _twist_contract_error(message, command):
    if not _twist_is_finite(message):
        return "nonfinite"
    unsupported = (
        message.linear.y, message.linear.z,
        message.angular.x, message.angular.y,
    )
    if any(value != 0.0 for value in unsupported):
        return "unsupported_axis"
    try:
        linear = command["linear"]
        angular = command["angular"]
        minimum_linear, maximum_linear = float(linear["minimum"]), float(linear["maximum"])
        minimum_angular, maximum_angular = float(angular["minimum"]), float(angular["maximum"])
    except (KeyError, TypeError, ValueError):
        return "invalid_bounds"
    if not (minimum_linear <= message.linear.x <= maximum_linear):
        return "linear_bounds"
    if not (minimum_angular <= message.angular.z <= maximum_angular):
        return "angular_bounds"
    return None


def main():
    args = _parse_args()
    evidence = {}

    # Contract, authority, and measured receipt evaluation precede every ROS import.
    try:
        manifest_path = os.path.abspath(args.manifest)
        authority_path = os.path.abspath(args.release_authority)
        manifest = load_manifest(manifest_path)
        contract_sha256 = _contract_hash(manifest_path)
        authority = {}
        if args.profile == "hardware_enabled":
            authority = load_manifest(authority_path)
            evidence = _load_runtime_evidence(
                args.bundle_root,
                args.runtime_evidence,
                args.runtime_evidence_sha256,
                manifest_path,
                authority_path,
            )
        result = _adapter_preflight(manifest, args.profile, evidence)
    except (DriverContractError, OSError, ValueError) as exc:
        print("hardware adapter preflight failed: {}".format(exc), file=sys.stderr)
        return 2
    if getattr(result, "errors", ()):
        errors = ",".join(getattr(result, "error_codes", ())) or "E_CONTRACT"
        print("hardware adapter preflight failed: {}".format(errors), file=sys.stderr)
        return 2

    enabled = _endpoint_authorized(
        manifest,
        authority,
        result,
        args.profile,
        _bool(args.hardware_enable),
        evidence,
    )
    if args.profile == "hardware_enabled" and not enabled:
        errors = ",".join(getattr(result, "error_codes", ())) or "E_NOT_AUTHORIZED"
        print("hardware adapter refused motor endpoint: {}".format(errors), file=sys.stderr)
        return 3

    command = manifest.get("command") or {}
    safe_topic = str(command.get("safe_input_topic") or "/cmd_vel_safe")
    if safe_topic != "/cmd_vel_safe":
        print("hardware adapter refused non-safe command input", file=sys.stderr)
        return 4

    driver_topic = ""
    if enabled:
        driver_topic = str(command.get("driver_topic") or "")
        message_type = str(command.get("message_type") or "")
        adapter_mode = str((manifest.get("adapter") or {}).get("mode") or "")
        if (adapter_mode != "direct_twist" or driver_topic != "/cmd_vel_safe"
                or message_type not in ("geometry_msgs/Twist", "geometry_msgs/msg/Twist")):
            print(
                "hardware adapter refused translated or non-direct endpoint; "
                "the manifest-selected driver must own /cmd_vel_safe directly",
                file=sys.stderr,
            )
            return 5
        try:
            mode_type = _evidence_message_class((manifest.get("mode") or {}).get("message_type"))
            override_type = _evidence_message_class(
                (manifest.get("manual_override") or {}).get("message_type")
            )
            estop_type = _evidence_message_class((manifest.get("estop") or {}).get("message_type"))
        except DriverContractError as exc:
            print("hardware adapter preflight failed: {}".format(exc), file=sys.stderr)
            return 5

    # ROS imports are lazy so contract tooling remains usable without ROS installed.
    import rospy
    from geometry_msgs.msg import Twist
    from wheelchair_interfaces.msg import DriverStatus, SafetySignal

    rospy.init_node("hardware_adapter", anonymous=False)
    status_publisher = rospy.Publisher(
        _CANONICAL_DRIVER_TOPIC, DriverStatus, queue_size=1, latch=True
    )
    mode_publisher = rospy.Publisher(_CANONICAL_MODE_TOPIC, SafetySignal, queue_size=1, latch=True)
    driver_signal_publisher = rospy.Publisher(
        _CANONICAL_DRIVER_SIGNAL_TOPIC, SafetySignal, queue_size=1, latch=True
    )
    # Direct mode is owned by the verified driver itself. This process never creates
    # a publisher on its own `/cmd_vel_safe` input and cannot become a command relay.

    sequence = [0]
    samples = {
        "mode": {"value": None, "stamp": None},
        "override": {"value": None, "stamp": None},
        "estop": {"value": None, "stamp": None},
    }
    source = "hardware_adapter:{}".format(args.profile)

    def publish_status(_event=None):
        now = rospy.Time.now()
        clear, fresh, mode_auto, override_active, estop_asserted, reason = _evidence_decision(
            manifest, enabled, samples, now
        )
        current_sequence = sequence[0]
        sequence[0] += 1

        status = DriverStatus()
        status.header.stamp = now
        status.sequence = current_sequence
        if not enabled or not fresh:
            status.state = DriverStatus.UNKNOWN
        elif estop_asserted:
            status.state = DriverStatus.FAULT
        elif override_active:
            status.state = DriverStatus.AUTO_DISABLED
        elif not mode_auto:
            status.state = DriverStatus.MANUAL
        else:
            status.state = DriverStatus.AUTO_READY
        status.reason_mask = reason
        status.source = source
        status.contract_id = str(manifest.get("contract_id") or "")
        status.contract_sha256 = contract_sha256
        status.enabled = clear
        status.manual_override_active = override_active
        status.physical_estop_asserted = estop_asserted
        status.watchdog_verified = clear
        status.heartbeat_age_s = (
            max((now - sample["stamp"]).to_sec() for sample in samples.values())
            if fresh
            else max(
                float((manifest.get("mode") or {}).get("stale_timeout_s") or 0.0),
                float((manifest.get("manual_override") or {}).get("stale_timeout_s") or 0.0),
                float((manifest.get("estop") or {}).get("stale_timeout_s") or 0.0),
            )
            + _STATUS_PERIOD_S
        )
        status.command_timeout_s = float(command.get("timeout_s") or 0.0)
        status.measured_linear_mps = -1.0
        status.measured_angular_rps = -1.0

        for publisher in (mode_publisher, driver_signal_publisher):
            signal = SafetySignal()
            signal.header.stamp = now
            signal.sequence = current_sequence
            signal.state = SafetySignal.CLEAR if clear else SafetySignal.STOP
            signal.reason_mask = reason
            signal.source = source
            signal.policy_sha256 = contract_sha256
            publisher.publish(signal)
        status_publisher.publish(status)

    def evidence_callback(name):
        def callback(message):
            samples[name] = {"value": message.data, "stamp": rospy.Time.now()}

        return callback

    def safe_command_callback(message):
        error = _twist_contract_error(message, command)
        if error is not None:
            rospy.logerr_throttle(
                1.0, "hardware shadow rejected /cmd_vel_safe command: %s", error
            )
            return
        rospy.loginfo_throttle(
            5.0,
            "hardware shadow observed safe command linear=%.3f angular=%.3f",
            message.linear.x,
            message.angular.z,
        )

    if not enabled:
        rospy.Subscriber(
            safe_topic, Twist, safe_command_callback, queue_size=1, tcp_nodelay=True
        )
    if enabled:
        rospy.Subscriber(
            manifest["mode"]["status_topic"], mode_type, evidence_callback("mode"), queue_size=1
        )
        rospy.Subscriber(
            manifest["manual_override"]["status_topic"],
            override_type,
            evidence_callback("override"),
            queue_size=1,
        )
        rospy.Subscriber(
            manifest["estop"]["status_topic"],
            estop_type,
            evidence_callback("estop"),
            queue_size=1,
        )
    rospy.Timer(rospy.Duration(_STATUS_PERIOD_S), publish_status)
    publish_status()
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
