from pathlib import Path
import re


PACKAGE = Path(__file__).resolve().parents[1]
MSG_DIR = PACKAGE / "msg"
ACTION_DIR = PACKAGE / "action"

REQUIRED_MESSAGES = (
    "SafetyReason.msg",
    "SafetySignal.msg",
    "CollisionStatus.msg",
    "SlopeStatus.msg",
    "LocalizationCandidate.msg",
    "LocalizationStatus.msg",
    "ActiveRoute.msg",
    "GeofenceStatus.msg",
    "MotionIntent.msg",
    "RouteProgress.msg",
    "MissionState.msg",
    "DriverStatus.msg",
    "SafetyState.msg",
)

SAFETY_REASONS = {
    "ESTOP": 1,
    "STALE_CMD": 2,
    "MODE": 4,
    "GEOFENCE": 8,
    "COLLISION": 16,
    "LOCALIZATION": 32,
    "DRIVER": 64,
    "INVALID_CMD": 128,
    "CLOCK": 256,
    "STALE_INTENT": 512,
    "INTERNAL_FAULT": 1024,
    "STARTUP": 2048,
    "SENSOR_STALE": 4096,
    "COLLISION_BLIND": 8192,
    "COLLISION_TTC": 16384,
    "COLLISION_DISTANCE": 32768,
    "SLOPE": 65536,
    "IMU_UNCALIBRATED": 131072,
    "ROUTE_MANIFEST": 262144,
    "GRAPH_TOPOLOGY": 524288,
    "TF": 1048576,
    "BACKPRESSURE": 2097152,
    "DEADLINE_MISS": 4194304,
    "MANUAL_OVERRIDE": 8388608,
    "HARDWARE_UNVERIFIED": 16777216,
    "MAP_MISMATCH": 33554432,
    "COLLISION_OCCLUDED": 67108864,
    "LOCALIZATION_INCONSISTENT": 134217728,
    "RESOURCE": 268435456,
    "CORRUPT_DATA": 536870912,
    "RESET_REJECTED": 1073741824,
    "INPUT_UNKNOWN": 2147483648,
    "ROUTE_STATE": 4294967296,
    "ODOM_STALE": 8589934592,
    "IMU_STALE": 17179869184,
    "LIDAR_STALE": 34359738368,
    "POLICY_MISMATCH": 68719476736,
}

ACTION_SECTIONS = (
    (
        "uint8 DIRECTION_OUTBOUND=1",
        "uint8 DIRECTION_RETURN=2",
        "string mission_id",
        "string route_id",
        "uint8 direction",
        "string map_id",
        "string map_sha256",
        "string route_manifest_sha256",
        "string safety_manifest_sha256",
    ),
    (
        "uint8 SUCCEEDED=0",
        "uint8 REJECTED=1",
        "uint8 CANCELED=2",
        "uint8 ABORTED=3",
        "uint8 FAULT=4",
        "bool success",
        "uint8 result_code",
        "uint64 reason_mask",
        "string message",
    ),
    (
        "wheelchair_interfaces/RouteProgress progress",
        "wheelchair_interfaces/MissionState mission_state",
    ),
)

EXPECTED_SHA_FIELDS = {
    "SafetyReason.msg": (),
    "SafetySignal.msg": ("string policy_sha256",),
    "CollisionStatus.msg": ("string policy_sha256",),
    "SlopeStatus.msg": ("string policy_sha256", "string calibration_sha256"),
    "LocalizationCandidate.msg": ("string map_sha256",),
    "LocalizationStatus.msg": ("string map_sha256", "string policy_sha256"),
    "ActiveRoute.msg": (
        "string map_sha256",
        "string route_manifest_sha256",
        "string safety_manifest_sha256",
    ),
    "GeofenceStatus.msg": ("string manifest_sha256",),
    "MotionIntent.msg": (),
    "RouteProgress.msg": (),
    "MissionState.msg": (),
    "DriverStatus.msg": ("string contract_sha256",),
    "SafetyState.msg": ("string release_manifest_sha256",),
    "ExecuteRoute.action": (
        "string map_sha256",
        "string route_manifest_sha256",
        "string safety_manifest_sha256",
    ),
}


def _source_lines(path):
    return tuple(
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    )


def _constants(path):
    constants = {}
    for line in _source_lines(path):
        match = re.fullmatch(r"uint(?:8|16|32|64) ([A-Z][A-Z0-9_]*)=([0-9]+)", line)
        if match:
            constants[match.group(1)] = int(match.group(2))
    return constants


def test_all_frozen_interface_sources_exist():
    assert {path.name for path in MSG_DIR.glob("*.msg")} == set(REQUIRED_MESSAGES)
    assert {path.name for path in ACTION_DIR.glob("*.action")} == {"ExecuteRoute.action"}


def test_execute_route_action_has_exact_goal_result_and_feedback_sections():
    lines = _source_lines(ACTION_DIR / "ExecuteRoute.action")
    assert lines.count("---") == 2
    separators = [index for index, line in enumerate(lines) if line == "---"]
    sections = (
        lines[: separators[0]],
        lines[separators[0] + 1 : separators[1]],
        lines[separators[1] + 1 :],
    )
    assert sections == ACTION_SECTIONS


def test_safety_reason_registry_is_the_exact_unique_numeric_bit_range():
    path = MSG_DIR / "SafetyReason.msg"
    constants = _constants(path)
    assert constants == SAFETY_REASONS
    assert len(set(constants.values())) == len(constants)
    assert tuple(constants.values()) == tuple(1 << bit for bit in range(37))
    assert _source_lines(path)[-1] == "uint64 mask"


def test_safety_reason_does_not_declare_reserved_bits():
    declarations = _source_lines(MSG_DIR / "SafetyReason.msg")
    assert all("RESERVED" not in line.upper() for line in declarations)
    assert max(SAFETY_REASONS.values()) == 1 << 36


def test_fail_closed_unknown_hold_and_disarmed_constants_are_zero():
    expected_zero = {
        "SafetySignal.msg": ("UNKNOWN",),
        "CollisionStatus.msg": ("STATE_UNKNOWN", "VISIBILITY_UNKNOWN"),
        "SlopeStatus.msg": ("STATE_UNKNOWN",),
        "GeofenceStatus.msg": ("UNKNOWN",),
        "MotionIntent.msg": ("HOLD",),
        "MissionState.msg": ("DISARMED",),
        "DriverStatus.msg": ("UNKNOWN",),
        "SafetyState.msg": ("DISARMED",),
    }
    for filename, names in expected_zero.items():
        constants = _constants(MSG_DIR / filename)
        for name in names:
            assert constants.get(name) == 0, f"{filename}: {name} must remain zero"

    for filename in set(REQUIRED_MESSAGES) - {"SafetyReason.msg"}:
        for name, value in _constants(MSG_DIR / filename).items():
            if "UNKNOWN" in name or name in {"HOLD", "DISARMED"}:
                assert value == 0, f"{filename}: {name} must fail closed at zero"


def test_hash_bearing_fields_are_exact_and_string_typed():
    paths = [MSG_DIR / name for name in REQUIRED_MESSAGES]
    paths.append(ACTION_DIR / "ExecuteRoute.action")
    for path in paths:
        actual = tuple(line for line in _source_lines(path) if line.split()[-1].endswith("sha256"))
        assert actual == EXPECTED_SHA_FIELDS[path.name]
