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
_RELEASE_MANIFEST_SCHEMA = "wheelchair-noetic-release-manifest/v2"
_RUNTIME_RECEIPT_FIELDS = (
    "schema_version", "status", "driver_manifest_path", "driver_manifest_sha256",
    "release_authority_path", "release_authority_sha256", "bundle_manifest_path",
    "bundle_manifest_sha256", "platform_receipt_path", "platform_receipt_sha256",
    "graph_receipt_path", "graph_receipt_sha256", "preflight_receipt_path",
    "preflight_receipt_sha256",
)
_RELEASE_INVENTORY_CATEGORIES = (
    "source_build_metadata", "package_metadata", "interfaces", "python_runtime",
    "configuration", "launch_configuration", "robot_assets", "contracts", "maps",
    "routes", "operator_docs", "ci_tools", "qualification_tools",
    "qualification_evidence",
)
_RELEASE_GATE_REPORTS = {
    "WP0-ABI-001": "evidence/contracts/abi-v1-report.json",
    "WP1-TOPOLOGY-001": "evidence/topology/command-graph-report.json",
    "WP1-GEOFENCE-001": "evidence/route-safety/anti-widening-report.json",
    "WP1-COLLISION-001": "evidence/safety/collision-ttc-report.json",
    "WP1-SLOPE-001": "evidence/safety/slope-policy-report.json",
    "WP1-CONTROL-001": "evidence/safety/gate-permission-matrix.json",
    "WP2-CONVERSION-001": "evidence/conversion/determinism-and-corruption-report.json",
    "WP3-LOCALIZATION-001": "evidence/localization/confidence-holdout-report.json",
    "WP3-GLIM-INPUT-001": "evidence/localization/glim-offline-input-report.json",
    "WP3-GLIM-REPRODUCTION-001": "evidence/localization/glim-offline-reproduction-report.json",
    "WP3-GLIM-COMPARISON-001": "evidence/localization/glim-offline-comparison-report.json",
    "WP4-MISSION-001": "evidence/mission/fsm-contract-report.json",
    "WP6-TIMING-001": "evidence/performance/target-nuc-60min-report.json",
    "WP6-SIMCLAIM-001": "evidence/simulation/fidelity-claim-report.json",
    "WP6-ROLLBACK-001": "evidence/release/rollback-drill-report.json",
    "WP0-HWGATE-NEG-001": "evidence/hardware/hardware-gate-negative-report.json",
    "WP0-PASSENGER-NEG-001": "evidence/release/passenger-authority-negative-report.json",
}
_RELEASE_AUTHORITY = {
    "software_release_candidate": True,
    "clean_release_authority": True,
    "hardware_motion_authorized": False,
    "passenger_operation_authorized": False,
    "physical_authority": False,
    "simulation_or_replay_is_physical_evidence": False,
}
_RELEASE_QUALIFICATION = {
    "target_nuc": "passed",
    "hardware": "blocked",
    "passenger": "blocked",
}
_RELEASE_RESIDUAL_BLOCKERS = [
    "hardware_motion_unqualified",
    "passenger_operation_unqualified",
]


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

def _canonical_hash(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    ).hexdigest()


def _strict_json(path, fields, label):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise DriverContractError("E_FORMAT", label + " is malformed") from exc
    if not isinstance(value, dict) or set(value) != set(fields):
        raise DriverContractError("E_FORMAT", label + " fields mismatch")
    return value


def _release_inventory(manifest):
    hashes = manifest.get("hashes")
    if not isinstance(hashes, dict) or set(hashes) != set(_RELEASE_INVENTORY_CATEGORIES):
        raise DriverContractError("E_FORMAT", "release inventory is malformed")
    inventory = {}
    digests = {}
    for name in _RELEASE_INVENTORY_CATEGORIES:
        category = hashes[name]
        if not isinstance(category, dict) or set(category) != {"digest", "files"}:
            raise DriverContractError("E_FORMAT", "release inventory category is malformed")
        files = category["files"]
        if not isinstance(files, list) or not files or files != sorted(
                files, key=lambda entry: entry.get("path", "") if isinstance(entry, dict) else ""):
            raise DriverContractError("E_FORMAT", "release inventory files are malformed")
        for entry in files:
            relative = entry.get("path") if isinstance(entry, dict) else None
            path = Path(relative) if isinstance(relative, str) else None
            if (not isinstance(entry, dict) or set(entry) != {"path", "sha256", "executable"}
                    or not isinstance(relative, str) or not relative or path.is_absolute()
                    or ".." in path.parts or not _sha256(entry["sha256"])
                    or type(entry["executable"]) is not bool or relative in inventory):
                raise DriverContractError("E_FORMAT", "release inventory entry is malformed")
            inventory[relative] = entry["sha256"]
        if not _sha256(category["digest"]) or category["digest"] != _canonical_hash(files):
            raise DriverContractError("E_FORMAT", "release inventory digest is malformed")
        digests[name] = category["digest"]
    return inventory, digests


def _release_manifest(path):
    release = _strict_json(
        path,
        ("schema", "source", "hashes", "gate_matrix", "authority", "qualification",
         "test_reports", "residual_blockers", "rollback", "release_binding_sha256",
         "release_signature_hmac_sha256"),
        "release manifest",
    )
    if (release["schema"] != _RELEASE_MANIFEST_SCHEMA
            or not _sha256(release["release_binding_sha256"])
            or not _sha256(release["release_signature_hmac_sha256"])):
        raise DriverContractError("E_FORMAT", "release manifest schema, binding, or signature is invalid")
    unsigned = dict(release)
    unsigned.pop("release_binding_sha256")
    unsigned.pop("release_signature_hmac_sha256")
    if _canonical_hash(unsigned) != release["release_binding_sha256"]:
        raise DriverContractError("E_FORMAT", "release manifest binding mismatch")
    source = release["source"]
    if (not isinstance(source, dict) or set(source) != {"kind", "revision", "worktree_clean"}
            or source["kind"] != "git_commit" or not isinstance(source["revision"], str)
            or len(source["revision"]) != 40
            or not all(c in "0123456789abcdef" for c in source["revision"])
            or source["worktree_clean"] is not True
            or release["authority"] != _RELEASE_AUTHORITY
            or release["qualification"] != _RELEASE_QUALIFICATION
            or release["residual_blockers"] != _RELEASE_RESIDUAL_BLOCKERS):
        raise DriverContractError("E_FORMAT", "release manifest authority is invalid")
    inventory, digests = _release_inventory(release)
    expected_gates = sorted(_RELEASE_GATE_REPORTS)
    bindings = {
        "sourceRevision": source["revision"],
        "configurationDigest": digests["configuration"],
        "releaseInputDigest": _canonical_hash({
            name: digests[name] for name in _RELEASE_INVENTORY_CATEGORIES
            if name != "qualification_evidence"
        }),
    }
    bindings["bundleDigest"] = _canonical_hash({
        "source_revision": source["revision"],
        "configuration_digest": bindings["configurationDigest"],
        "release_input_digest": bindings["releaseInputDigest"],
    })
    matrix = release["gate_matrix"]
    if (not isinstance(matrix, dict) or set(matrix) != {
                "requiredGateIds", "passedGateIds", "releaseBindings"}
            or matrix["requiredGateIds"] != expected_gates
            or matrix["passedGateIds"] != expected_gates
            or matrix["releaseBindings"] != bindings):
        raise DriverContractError("E_FORMAT", "release gate matrix is incomplete or stale")
    reports = release["test_reports"]
    expected_paths = sorted(_RELEASE_GATE_REPORTS.values())
    if (not isinstance(reports, list) or len(reports) != len(expected_paths)
            or [entry.get("path") if isinstance(entry, dict) else None for entry in reports]
            != expected_paths):
        raise DriverContractError("E_FORMAT", "release test reports are malformed")
    for entry in reports:
        if (not isinstance(entry, dict) or set(entry) != {"path", "sha256", "executable"}
                or entry["executable"] is not False or not _sha256(entry["sha256"])
                or inventory.get(entry["path"]) != entry["sha256"]):
            raise DriverContractError("E_FORMAT", "release test reports are not inventoried")
    rollback = release["rollback"]
    rollback_fields = {
        "parentReleaseBindingSha256", "parentManifestSha256", "parentManifestPath",
        "parentInventoryDigest", "restartReceipt",
    }
    if (not isinstance(rollback, dict) or set(rollback) != rollback_fields
            or not all(isinstance(rollback[key], str) and rollback[key]
                       for key in rollback_fields - {"restartReceipt"})
            or not all(_sha256(rollback[key]) for key in (
                "parentReleaseBindingSha256", "parentManifestSha256", "parentInventoryDigest"))
            or Path(rollback["parentManifestPath"]).is_absolute()
            or ".." in Path(rollback["parentManifestPath"]).parts):
        raise DriverContractError("E_FORMAT", "release rollback binding is malformed")
    restart = rollback["restartReceipt"]
    if (not isinstance(restart, dict) or set(restart) != {
                "path", "sha256", "parentReleaseBindingSha256", "parentInventoryDigest"}
            or not all(isinstance(restart[key], str) and restart[key] for key in restart)
            or not _sha256(restart["sha256"])
            or restart["parentReleaseBindingSha256"] != rollback["parentReleaseBindingSha256"]
            or restart["parentInventoryDigest"] != rollback["parentInventoryDigest"]):
        raise DriverContractError("E_FORMAT", "release rollback receipt is malformed")
    return release, inventory


def _measured_platform(receipt, manifest):
    fields = ("schema_version", "artifact_type", "manufacturer", "model", "serial",
              "nuc_machine_id_sha256", "base_model_source_sha256")
    if (set(receipt) != set(fields) or receipt["schema_version"] != 1
            or receipt["artifact_type"] != "wheelchair-platform-measurement/v1"):
        raise DriverContractError("E_FORMAT", "platform receipt schema mismatch")
    platform = manifest.get("platform")
    if not isinstance(platform, dict) or any(receipt[key] != platform.get(key) for key in fields[2:]):
        raise DriverContractError("E_FORMAT", "measured platform identity mismatch")


def _measured_graph(receipt, manifest):
    fields = ("schema_version", "artifact_type", "graph_snapshot_sha256",
              "command_polarity_report_sha256", "timeout_report_sha256",
              "mode_override_report_sha256", "estop_report_sha256", "odom_tf_report_sha256")
    if (set(receipt) != set(fields) or receipt["schema_version"] != 1
            or receipt["artifact_type"] != "wheelchair-graph-measurement/v1"):
        raise DriverContractError("E_FORMAT", "graph receipt schema mismatch")
    evidence = manifest.get("evidence")
    if not isinstance(evidence, dict) or any(receipt[key] != evidence.get(key) for key in fields[2:]):
        raise DriverContractError("E_FORMAT", "measured graph identity mismatch")

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
    """Derive runtime matches from release-inventoried raw measurements."""
    if not _sha256(receipt_sha256):
        raise DriverContractError("E_FORMAT", "runtime evidence SHA-256 is invalid")
    root = Path(bundle_root)
    receipt_path = _contained_regular_file(root, receipt_relative)
    if _contract_hash(receipt_path) != receipt_sha256:
        raise DriverContractError("E_FORMAT", "runtime evidence SHA-256 mismatch")
    receipt = _strict_json(receipt_path, _RUNTIME_RECEIPT_FIELDS, "runtime evidence")
    if receipt["schema_version"] != 2 or receipt["status"] != "verified":
        raise DriverContractError("E_FORMAT", "runtime evidence is not verified")

    expected_paths = {
        "driver_manifest": Path(manifest_path).resolve(strict=True),
        "release_authority": Path(authority_path).resolve(strict=True),
    }
    bound = {}
    for prefix in ("driver_manifest", "release_authority", "bundle_manifest",
                   "platform_receipt", "graph_receipt", "preflight_receipt"):
        relative, digest = receipt[prefix + "_path"], receipt[prefix + "_sha256"]
        if not isinstance(relative, str) or not _sha256(digest):
            raise DriverContractError("E_FORMAT", prefix + " binding is invalid")
        candidate = _contained_regular_file(root, relative)
        if prefix in expected_paths and candidate.resolve(strict=True) != expected_paths[prefix]:
            raise DriverContractError("E_FORMAT", prefix + " path mismatch")
        if _contract_hash(candidate) != digest:
            raise DriverContractError("E_FORMAT", prefix + " SHA-256 mismatch")
        bound[prefix] = (relative, digest, candidate)

    release, inventory = _release_manifest(bound["bundle_manifest"][2])
    for prefix, (relative, digest, _candidate) in bound.items():
        if prefix == "bundle_manifest":
            continue
        if inventory.get(relative) != digest:
            raise DriverContractError("E_FORMAT", prefix + " is absent from release inventory")

    manifest = load_manifest(manifest_path)
    release_authority = load_manifest(authority_path)
    scope = release_authority.get("release_scope") or {}
    if (scope.get("hardware_motion_authorized") is not False
            or scope.get("passenger_operation_authorized") is not False
            or (release_authority.get("blocked_profiles") or {}).get(
                "hardware_enabled", {}).get("allowed") is not False):
        raise DriverContractError("E_FORMAT", "release authority is not fail-closed")
    _measured_platform(
        _strict_json(bound["platform_receipt"][2],
                     ("schema_version", "artifact_type", "manufacturer", "model", "serial",
                      "nuc_machine_id_sha256", "base_model_source_sha256"),
                     "platform receipt"),
        manifest,
    )
    _measured_graph(
        _strict_json(bound["graph_receipt"][2],
                     ("schema_version", "artifact_type", "graph_snapshot_sha256",
                      "command_polarity_report_sha256", "timeout_report_sha256",
                      "mode_override_report_sha256", "estop_report_sha256", "odom_tf_report_sha256"),
                     "graph receipt"),
        manifest,
    )
    preflight = _strict_json(
        bound["preflight_receipt"][2],
        ("schema_version", "artifact_type", "driver_manifest_sha256", "release_authority_sha256",
         "platform_receipt_sha256", "graph_receipt_sha256", "release_binding_sha256"),
        "preflight receipt",
    )
    if (preflight["schema_version"] != 1
            or preflight["artifact_type"] != "wheelchair-driver-preflight/v1"
            or any(preflight[key] != receipt[key] for key in (
                "driver_manifest_sha256", "release_authority_sha256",
                "platform_receipt_sha256", "graph_receipt_sha256"))
            or preflight["release_binding_sha256"] != release["release_binding_sha256"]):
        raise DriverContractError("E_FORMAT", "preflight receipt binding mismatch")
    return {
        "platform_matches": True,
        "base_model_matches": True,
        "graph_valid": True,
        "receipt_verified": True,
    }


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
        if (manifest.get("adapter") or {}).get("mode") == "translated":
            raise DriverContractError(
                "E_ADAPTER_MODE",
                "translated mode is unsupported without a manifest-pinned translator implementation",
            )
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
