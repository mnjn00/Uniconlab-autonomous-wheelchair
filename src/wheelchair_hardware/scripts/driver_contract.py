#!/usr/bin/env python3
"""ROS-independent, fail-closed validation for the hardware driver boundary."""

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import jsonschema
import yaml


PROFILES = ("sim", "replay", "hardware_shadow", "hardware_enabled")
RUNTIME_EVIDENCE_FLAGS = ("platform_matches", "base_model_matches", "graph_valid")


@dataclass(frozen=True)
class ContractIssue:
    code: str
    path: str
    message: str


class DriverContractError(ValueError):
    """A manifest could not be loaded safely."""

    def __init__(self, code: str, message: str, path: str = "$") -> None:
        super().__init__("{} {}: {}".format(code, path, message))
        self.code = code
        self.path = path
        self.message = message


@dataclass(frozen=True)
class HardwarePreflight:
    profile: str
    allowed: bool
    deployable: bool
    real_motor_path: bool
    driver_topic: str
    adapter_mode: str
    errors: Tuple[ContractIssue, ...]

    @property
    def error_codes(self) -> Tuple[str, ...]:
        return tuple(issue.code for issue in self.errors)


def load_manifest(path: os.PathLike) -> Dict[str, Any]:
    """Load one YAML manifest without ROS or object construction side effects."""
    try:
        with open(path, "r", encoding="utf-8") as stream:
            value = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise DriverContractError("E_FORMAT", str(exc)) from exc
    if not isinstance(value, dict):
        raise DriverContractError("E_FORMAT", "manifest root must be a mapping")
    return value


def _find_schema(explicit: Optional[os.PathLike]) -> Path:
    if explicit is not None:
        candidate = Path(explicit)
        if candidate.is_file():
            return candidate
        raise DriverContractError("E_FORMAT", "driver schema does not exist: {}".format(candidate))

    configured = os.environ.get("WHEELCHAIR_DRIVER_SCHEMA")
    if configured:
        return _find_schema(configured)

    starts = (Path(__file__).resolve(), Path.cwd().resolve())
    for start in starts:
        for parent in (start,) + tuple(start.parents):
            candidate = parent / "contracts" / "wp0" / "A11-driver-contract-schema.json"
            if candidate.is_file():
                return candidate
    raise DriverContractError("E_FORMAT", "A11 driver schema was not found")


def _json_path(parts: Sequence[Any]) -> str:
    return "$" + "".join("[{}]".format(part) if isinstance(part, int) else ".{}".format(part) for part in parts)


def _schema_error_code(error: jsonschema.ValidationError, manifest: Mapping[str, Any]) -> str:
    path = tuple(error.absolute_path)
    root = path[0] if path else ""
    leaf = path[-1] if path else ""
    if root == "schema_version" or (error.validator == "required" and "schema_version" in error.message):
        return "E_SCHEMA_VERSION"
    if error.validator == "additionalProperties":
        return "E_UNKNOWN_FIELD"
    if error.validator == "required":
        return "E_REQUIRED"
    if manifest.get("verified") is False and error.validator == "const":
        return "E_UNVERIFIED"
    if root == "adapter":
        return "E_TRANSLATION_SPEC" if leaf == "translation_spec_sha256" else "E_ADAPTER_MODE"
    if root == "command":
        return {
            "safe_input_topic": "E_SAFE_INPUT", "driver_topic": "E_DRIVER_TOPIC_MISSING",
            "message_type": "E_DRIVER_TYPE", "message_md5": "E_DRIVER_MD5",
            "publish_rate_hz": "E_RATE", "timeout_s": "E_TIMEOUT",
            "timeout_owner": "E_TIMEOUT_OWNER", "timeout_behavior": "E_TIMEOUT_BEHAVIOR",
            "linear": "E_AXIS", "angular": "E_AXIS",
        }.get(leaf, "E_AXIS" if "linear" in path or "angular" in path else "E_FORMAT")
    return {
        "mode": "E_MODE_STATUS", "manual_override": "E_MANUAL_OVERRIDE",
        "estop": "E_ESTOP_CHAIN", "odometry": "E_ODOM_TF",
        "evidence": "E_EVIDENCE_HASH", "approval": "E_APPROVAL",
    }.get(root, "E_FORMAT")


def _append(issues: list, code: str, path: str, message: str) -> None:
    issue = ContractIssue(code, path, message)
    if issue not in issues:
        issues.append(issue)


def _walk_strings(value: Any, path: str = "$"):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from _walk_strings(child, "{}.{}".format(path, key))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from _walk_strings(child, "{}[{}]".format(path, index))
    elif isinstance(value, str):
        yield path, value


def validate_manifest(manifest: Any, schema_path: Optional[os.PathLike] = None) -> Tuple[ContractIssue, ...]:
    """Return stable validation issues; an empty tuple means schema/semantic validity."""
    if not isinstance(manifest, dict):
        return (ContractIssue("E_FORMAT", "$", "manifest root must be a mapping"),)

    issues = []
    try:
        with open(_find_schema(schema_path), "r", encoding="utf-8") as stream:
            schema = json.load(stream)
        validator = jsonschema.Draft7Validator(schema, format_checker=jsonschema.FormatChecker())
    except (OSError, ValueError, json.JSONDecodeError, jsonschema.SchemaError, DriverContractError) as exc:
        return (ContractIssue("E_FORMAT", "$", "schema unavailable or invalid: {}".format(exc)),)

    for error in sorted(validator.iter_errors(manifest), key=lambda item: (list(item.absolute_path), item.message)):
        _append(issues, _schema_error_code(error, manifest), _json_path(error.absolute_path), error.message)

    for path, value in _walk_strings(manifest):
        if "cmd_vel_nav" in value:
            _append(issues, "E_NAV_BYPASS", path, "/cmd_vel_nav is forbidden at the hardware boundary")

    command = manifest.get("command")
    if isinstance(command, dict):
        for name in ("publish_rate_hz", "timeout_s"):
            value = command.get(name)
            if isinstance(value, (int, float)) and not isinstance(value, bool) and not math.isfinite(value):
                _append(issues, "E_RATE" if name == "publish_rate_hz" else "E_TIMEOUT", "$.command.{}".format(name), "value must be finite")
        for axis_name in ("linear", "angular"):
            axis = command.get(axis_name)
            if not isinstance(axis, dict):
                continue
            minimum, maximum = axis.get("minimum"), axis.get("maximum")
            if any(isinstance(value, (int, float)) and not isinstance(value, bool) and not math.isfinite(value) for value in (minimum, maximum)):
                _append(issues, "E_AXIS", "$.command.{}".format(axis_name), "axis limits must be finite")
            elif all(isinstance(value, (int, float)) and not isinstance(value, bool) for value in (minimum, maximum)):
                if minimum > maximum or (manifest.get("verified") is True and not (minimum < 0 < maximum)):
                    _append(issues, "E_AXIS", "$.command.{}".format(axis_name), "verified limits must be ordered and span zero")

    mode = manifest.get("mode")
    if isinstance(mode, dict) and manifest.get("verified") is True:
        if mode.get("auto_value") == mode.get("manual_value"):
            _append(issues, "E_MODE_STATUS", "$.mode", "auto and manual status values must be distinct")
    for section, field, code in (
        ("mode", "stale_timeout_s", "E_MODE_STATUS"),
        ("manual_override", "stale_timeout_s", "E_MANUAL_OVERRIDE"),
        ("estop", "stale_timeout_s", "E_ESTOP_CHAIN"),
        ("odometry", "minimum_rate_hz", "E_ODOM_TF"),
        ("odometry", "stale_timeout_s", "E_ODOM_TF"),
    ):
        block = manifest.get(section)
        value = block.get(field) if isinstance(block, dict) else None
        if isinstance(value, (int, float)) and not isinstance(value, bool) and not math.isfinite(value):
            _append(issues, code, "$.{}.{}".format(section, field), "value must be finite")

    return tuple(issues)


def preflight(
    manifest: Any,
    profile: str,
    runtime_evidence: Optional[Mapping[str, Any]] = None,
    schema_path: Optional[os.PathLike] = None,
) -> HardwarePreflight:
    """Decide whether a profile may start before any real endpoint is created."""
    issues = list(validate_manifest(manifest, schema_path))
    profile_known = profile in PROFILES
    if not profile_known:
        _append(issues, "E_FORMAT", "$.profile", "unknown launch profile: {}".format(profile))

    adapter = manifest.get("adapter", {}) if isinstance(manifest, dict) else {}
    command = manifest.get("command", {}) if isinstance(manifest, dict) else {}
    adapter_mode = adapter.get("mode", "disabled") if isinstance(adapter, dict) else "disabled"

    if profile != "hardware_enabled":
        return HardwarePreflight(profile, profile_known, False, False, "", adapter_mode, tuple(issues))

    if not isinstance(manifest, dict) or manifest.get("verified") is not True:
        _append(issues, "E_UNVERIFIED", "$.verified", "hardware requires a verified contract")
    if isinstance(manifest, dict) and manifest.get("example_only") is not False:
        _append(issues, "E_EXAMPLE_ONLY", "$.example_only", "example contracts cannot authorize hardware")
    if not isinstance(manifest, dict) or manifest.get("hardware_motion_authorized") is not True or manifest.get("passenger_operation_authorized") is not True:
        _append(issues, "E_AUTHORIZATION_FALSE", "$", "hardware and passenger authorization must both be explicitly true")

    evidence = runtime_evidence if isinstance(runtime_evidence, Mapping) else {}
    runtime_codes = {
        "platform_matches": "E_PLATFORM_MISMATCH",
        "base_model_matches": "E_BASE_MODEL_MISMATCH",
        "graph_valid": "E_GRAPH",
    }
    for flag in RUNTIME_EVIDENCE_FLAGS:
        if evidence.get(flag) is not True:
            _append(issues, runtime_codes[flag], "$.runtime_evidence.{}".format(flag), "explicit true runtime evidence is required")

    topic = command.get("driver_topic", "") if isinstance(command, dict) else ""
    deployable = profile_known and not issues and adapter_mode in ("direct_twist", "translated") and bool(topic)
    return HardwarePreflight(
        profile, deployable, deployable, deployable, topic if deployable else "", adapter_mode, tuple(issues)
    )


__all__ = [
    "ContractIssue", "DriverContractError", "HardwarePreflight", "PROFILES",
    "RUNTIME_EVIDENCE_FLAGS", "load_manifest", "validate_manifest", "preflight",
]
