from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
INTERFACES = ROOT / "src" / "wheelchair_interfaces"

REQUIRED_SOURCES = {
    "msg/SafetyReason.msg",
    "msg/SafetySignal.msg",
    "msg/CollisionStatus.msg",
    "msg/SlopeStatus.msg",
    "msg/LocalizationCandidate.msg",
    "msg/LocalizationStatus.msg",
    "msg/ActiveRoute.msg",
    "msg/GeofenceStatus.msg",
    "msg/MotionIntent.msg",
    "msg/RouteProgress.msg",
    "msg/MissionState.msg",
    "msg/DriverStatus.msg",
    "msg/SafetyState.msg",
    "action/ExecuteRoute.action",
}

EXPECTED_ACTION = """uint8 DIRECTION_OUTBOUND=1
uint8 DIRECTION_RETURN=2
string mission_id
string route_id
uint8 direction
string map_id
string map_sha256
string route_manifest_sha256
string safety_manifest_sha256
---
uint8 SUCCEEDED=0
uint8 REJECTED=1
uint8 CANCELED=2
uint8 ABORTED=3
uint8 FAULT=4
bool success
uint8 result_code
uint64 reason_mask
string message
---
wheelchair_interfaces/RouteProgress progress
wheelchair_interfaces/MissionState mission_state
"""

EXPECTED_HASH_DECLARATIONS = {
    ("msg/SafetySignal.msg", "string policy_sha256"),
    ("msg/CollisionStatus.msg", "string policy_sha256"),
    ("msg/SlopeStatus.msg", "string policy_sha256"),
    ("msg/SlopeStatus.msg", "string calibration_sha256"),
    ("msg/LocalizationCandidate.msg", "string map_sha256"),
    ("msg/LocalizationStatus.msg", "string map_sha256"),
    ("msg/LocalizationStatus.msg", "string policy_sha256"),
    ("msg/ActiveRoute.msg", "string map_sha256"),
    ("msg/ActiveRoute.msg", "string route_manifest_sha256"),
    ("msg/ActiveRoute.msg", "string safety_manifest_sha256"),
    ("msg/GeofenceStatus.msg", "string manifest_sha256"),
    ("msg/DriverStatus.msg", "string contract_sha256"),
    ("msg/SafetyState.msg", "string release_manifest_sha256"),
    ("action/ExecuteRoute.action", "string map_sha256"),
    ("action/ExecuteRoute.action", "string route_manifest_sha256"),
    ("action/ExecuteRoute.action", "string safety_manifest_sha256"),
}


def _lines(relative_path):
    return tuple(
        line.strip()
        for line in (INTERFACES / relative_path).read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    )


def _numeric_constants(relative_path):
    result = {}
    for line in _lines(relative_path):
        match = re.fullmatch(r"uint(?:8|16|32|64) ([A-Z][A-Z0-9_]*)=([0-9]+)", line)
        if match:
            result[match.group(1)] = int(match.group(2))
    return result


def test_appendix_a_source_set_is_complete():
    missing = sorted(path for path in REQUIRED_SOURCES if not (INTERFACES / path).is_file())
    assert not missing, f"missing frozen interface definitions: {missing}"


def test_execute_route_is_byte_for_byte_appendix_a_definition():
    actual = (INTERFACES / "action/ExecuteRoute.action").read_text(encoding="utf-8")
    assert actual == EXPECTED_ACTION
    assert actual.splitlines().count("---") == 2


def test_safety_reason_uses_each_bit_zero_through_thirty_six_once():
    constants = _numeric_constants("msg/SafetyReason.msg")
    assert list(constants.values()) == [1 << bit for bit in range(37)]
    assert len(constants) == len(set(constants)) == 37
    assert _lines("msg/SafetyReason.msg")[-1] == "uint64 mask"
    assert not any("RESERVED" in declaration.upper() for declaration in _lines("msg/SafetyReason.msg"))


def test_authority_states_keep_fail_closed_zero_encoding():
    zero_constants = {
        "msg/SafetySignal.msg": ("UNKNOWN",),
        "msg/CollisionStatus.msg": ("STATE_UNKNOWN", "VISIBILITY_UNKNOWN"),
        "msg/SlopeStatus.msg": ("STATE_UNKNOWN",),
        "msg/GeofenceStatus.msg": ("UNKNOWN",),
        "msg/MotionIntent.msg": ("HOLD",),
        "msg/MissionState.msg": ("DISARMED",),
        "msg/DriverStatus.msg": ("UNKNOWN",),
        "msg/SafetyState.msg": ("DISARMED",),
    }
    for path, names in zero_constants.items():
        constants = _numeric_constants(path)
        assert {name: constants.get(name) for name in names} == {name: 0 for name in names}


def test_hash_bearing_fields_cannot_be_renamed_or_retyped():
    actual = set()
    for relative_path in REQUIRED_SOURCES:
        for line in _lines(relative_path):
            if line.split()[-1].endswith("sha256"):
                actual.add((relative_path, line))
    assert actual == EXPECTED_HASH_DECLARATIONS
