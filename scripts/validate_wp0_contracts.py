#!/usr/bin/env python3
"""Fail-closed validation for the immutable WP0 software contracts."""

import argparse
import hashlib
import json
import re
import struct
import sys
from pathlib import Path

CONSENSUS_PLAN_SHA256 = "c8148c5f5d03a646c839e2966ee7fb5c57a433ab34a8cbfcce1d5ab69cc69068"
ARCHITECT_REVIEW_SHA256 = "7d91bb0e2182e38fc30b74cb096ab7ff2ea590ebe62934a5e6f8125bba2f8548"
CRITIC_REVIEW_SHA256 = "3844ab8a0594b888340459c57393c42bbd084c1282a34716ea6bdbfc59958d3a"
INTENT_RECONCILIATION_SHA256 = "a31092d64375ae1958fa77ea8adb682ffd1232d11f01865109fcb0112fb3607f"
AUTHORITY_FALSE_KEYS = {
    "hardware_authorized", "hardware_motion_authorized",
    "passenger_authorized", "passenger_operation_authorized",
    "campus_operation_authorized", "physical_authority",
    "real_motor_path_allowed", "transferable_to_hardware",
}
try:
    import yaml
except ImportError:  # pragma: no cover - exercised in an environment without PyYAML
    yaml = None

PLAN_SHA256 = "bd1b9454bc34f68714e6b986e80466535f817a42c25f662db0990adc79ca601e"
ABI_SHA256 = "2f9185da216397708649931207018f4fc8ed79ea1b2c4d3494afafa0891daed0"
ABI_INVENTORY = [
    "msg/SafetyReason.msg", "msg/SafetySignal.msg", "msg/CollisionStatus.msg",
    "msg/SlopeStatus.msg", "msg/LocalizationCandidate.msg",
    "msg/LocalizationStatus.msg", "msg/ActiveRoute.msg",
    "msg/GeofenceStatus.msg", "msg/MotionIntent.msg", "msg/RouteProgress.msg",
    "msg/MissionState.msg", "msg/DriverStatus.msg", "msg/SafetyState.msg",
    "action/ExecuteRoute.action",
]
REASONS = [
    "ESTOP", "STALE_CMD", "MODE", "GEOFENCE", "COLLISION", "LOCALIZATION",
    "DRIVER", "INVALID_CMD", "CLOCK", "STALE_INTENT", "INTERNAL_FAULT",
    "STARTUP", "SENSOR_STALE", "COLLISION_BLIND", "COLLISION_TTC",
    "COLLISION_DISTANCE", "SLOPE", "IMU_UNCALIBRATED", "ROUTE_MANIFEST",
    "GRAPH_TOPOLOGY", "TF", "BACKPRESSURE", "DEADLINE_MISS",
    "MANUAL_OVERRIDE", "HARDWARE_UNVERIFIED", "MAP_MISMATCH",
    "COLLISION_OCCLUDED", "LOCALIZATION_INCONSISTENT", "RESOURCE",
    "CORRUPT_DATA", "RESET_REJECTED", "INPUT_UNKNOWN", "ROUTE_STATE",
    "ODOM_STALE", "IMU_STALE", "LIDAR_STALE", "POLICY_MISMATCH",
]
REQUIRED_ARTIFACTS = [
    "A00-review-lineage.md", "A01-adr-noetic-localization.md",
    "A02-interface-abi-v1.md", "A03-topic-tf-time-ownership.yaml",
    "A04-safety-reason-registry.yaml", "A05-route-schema.json",
    "A06-route-safety-schema.json", "A07-collision-policy-schema.json",
    "A08-slope-policy-schema.json", "A09-localization-confidence-schema.json",
    "A10-conversion-abi-v1.md", "A11-driver-contract-schema.json",
    "A12-target-nuc-method.yaml", "A13-simulator-fidelity.yaml",
    "A14-hazard-log.yaml", "A15-evidence-inventory.yaml",
    "A16-release-authority.yaml", "A17-verification-matrix.yaml",
    "collision-simulation-policy.yaml", "driver-unverified.yaml",
    "driver-verified-fixture.yaml", "route-safety-candidate.yaml",
    "slope-simulation-policy.yaml", "time-alignment-schema.json",
]
SCHEMA_BINDINGS = {
    "collision-simulation-policy.yaml": "A07-collision-policy-schema.json",
    "slope-simulation-policy.yaml": "A08-slope-policy-schema.json",
    "route-safety-candidate.yaml": "A06-route-safety-schema.json",
    "driver-unverified.yaml": "A11-driver-contract-schema.json",
    "driver-verified-fixture.yaml": "A11-driver-contract-schema.json",
}
FALLBACK_REQUIRED = {
    "collision-simulation-policy.yaml": {"schema_version", "policy_id", "qualification", "hashes", "authority", "fail_closed"},
    "slope-simulation-policy.yaml": {"schema_version", "policy_id", "qualification", "hashes", "authority", "fail_closed"},
    "route-safety-candidate.yaml": {"schema_version", "manifest_id", "provenance", "approved_routes", "localization_zones", "authority"},
    "driver-unverified.yaml": {"schema_version", "verified", "example_only", "hardware_motion_authorized", "passenger_operation_authorized", "adapter", "command"},
    "driver-verified-fixture.yaml": {"schema_version", "verified", "example_only", "hardware_motion_authorized", "passenger_operation_authorized", "adapter", "command"},
}
HEX64 = re.compile(r"^[0-9a-f]{64}$")


class ContractError(Exception):
    def __init__(self, code, artifact, detail):
        self.code, self.artifact, self.detail = code, artifact, detail
        super().__init__(f"{code}: {artifact}: {detail}")


def require(condition, code, artifact, detail):
    if not condition:
        raise ContractError(code, artifact, detail)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_yaml(path):
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ContractError("E_YAML", path.name, str(exc))
    require(isinstance(value, dict), "E_YAML", path.name, "document root must be a mapping")
    return value


def load_json(path):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ContractError("E_JSON", path.name, str(exc))
    require(isinstance(value, dict), "E_JSON", path.name, "document root must be an object")
    return value


def nested(document, keys, artifact):
    value = document
    for key in keys:
        require(isinstance(value, dict) and key in value, "E_REQUIRED", artifact, "missing " + ".".join(keys))
        value = value[key]
    return value


def validate_manifest(contract_dir, documents):
    artifact = "manifest.yaml"
    manifest = documents[artifact]
    require(manifest.get("schema_version") == 1, "E_MANIFEST_FORMAT", artifact, "schema_version must be 1")
    require(manifest.get("algorithm") == "sha256", "E_MANIFEST_FORMAT", artifact, "algorithm must be sha256")
    entries = manifest.get("artifacts")
    require(isinstance(entries, list), "E_MANIFEST_FORMAT", artifact, "artifacts must be a list")
    paths = [entry.get("path") for entry in entries if isinstance(entry, dict)]
    require(len(paths) == len(entries) and len(paths) == len(set(paths)), "E_MANIFEST_FORMAT", artifact, "artifact paths must be mappings and unique")
    require(paths == REQUIRED_ARTIFACTS, "E_MANIFEST_SCOPE", artifact, "artifact inventory or order differs from the closed WP0 inventory")
    actual_files = sorted(path.name for path in contract_dir.iterdir() if path.is_file() and path.name != artifact)
    require(actual_files == sorted(REQUIRED_ARTIFACTS), "E_MANIFEST_SCOPE", artifact, "contract directory contains a missing or unlisted artifact")
    for entry in entries:
        expected = entry.get("sha256")
        require(isinstance(expected, str) and HEX64.fullmatch(expected), "E_MANIFEST_FORMAT", entry["path"], "sha256 must be 64 lowercase hex characters")
        observed = sha256(contract_dir / entry["path"])
        require(observed == expected, "E_MANIFEST_HASH", entry["path"], f"expected {expected}, observed {observed}")


def require_local_references(value, schema_name):
    if isinstance(value, dict):
        reference = value.get("$ref")
        require(
            reference is None or (isinstance(reference, str) and reference.startswith("#")),
            "E_SCHEMA",
            schema_name,
            "only local fragment $ref values are permitted",
        )
        for child in value.values():
            require_local_references(child, schema_name)
    elif isinstance(value, list):
        for child in value:
            require_local_references(child, schema_name)


def validate_schemas(contract_dir, documents):
    try:
        import jsonschema
    except ImportError:
        jsonschema = None
    for candidate_name, schema_name in SCHEMA_BINDINGS.items():
        candidate, schema = documents[candidate_name], documents[schema_name]
        if candidate_name in {"collision-simulation-policy.yaml", "slope-simulation-policy.yaml"}:
            expected_schema_hash = nested(candidate, ["hashes", "schema_sha256", "value"], candidate_name)
            require(
                expected_schema_hash == sha256(contract_dir / schema_name),
                "E_SCHEMA_HASH",
                candidate_name,
                f"schema hash does not match {schema_name}",
            )
        require(schema.get("$schema") == "http://json-schema.org/draft-07/schema#", "E_SCHEMA_DRAFT", schema_name, "must reference draft-07")
        require_local_references(schema, schema_name)
        if jsonschema is not None:
            try:
                jsonschema.Draft7Validator.check_schema(schema)
                validation_schema = dict(schema)
                validation_schema.pop("$id", None)
                errors = sorted(
                    jsonschema.Draft7Validator(validation_schema).iter_errors(candidate),
                    key=lambda error: tuple(str(part) for part in error.absolute_path),
                )
            except Exception as exc:
                raise ContractError("E_SCHEMA", schema_name, str(exc))
            if errors:
                error = errors[0]
                location = ".".join(str(part) for part in error.absolute_path) or "<root>"
                raise ContractError("E_SCHEMA", candidate_name, f"{location}: {error.message}")
        else:
            missing = sorted(FALLBACK_REQUIRED[candidate_name] - set(candidate))
            require(not missing, "E_REQUIRED", candidate_name, "missing top-level keys: " + ", ".join(missing))


def validate_abi(contract_dir):
    artifact = "A02-interface-abi-v1.md"
    text = (contract_dir / artifact).read_text(encoding="utf-8")
    declared = re.search(r"^canonical_abi_sha256: ([0-9a-f]{64})$", text, re.MULTILINE)
    require(declared and declared.group(1) == ABI_SHA256, "E_ABI_HASH", artifact, "canonical_abi_sha256 differs from ABI v1")
    inventory = re.findall(r"^\d+\. `((?:msg/.+\.msg)|(?:action/.+\.action))`$", text, re.MULTILINE)
    require(inventory == ABI_INVENTORY, "E_ABI_INVENTORY", artifact, "canonical interface inventory differs from ABI v1")
    blocks = re.findall(r"^### `([^`]+)`\n```text\n(.*?)```$", text, re.MULTILINE | re.DOTALL)
    require([path for path, _ in blocks] == ABI_INVENTORY, "E_ABI_INVENTORY", artifact, "canonical source blocks differ from inventory")
    payload = bytearray()
    for path, content in blocks:
        encoded_path = path.encode("utf-8")
        encoded_content = content.encode("utf-8")
        require(encoded_content.endswith(b"\n") and not encoded_content.endswith(b"\n\n"), "E_ABI_FORMAT", artifact, f"{path} must end in exactly one LF")
        payload.extend(encoded_path + b"\0" + struct.pack(">Q", len(encoded_content)) + encoded_content)
    observed = hashlib.sha256(payload).hexdigest()
    require(observed == ABI_SHA256, "E_ABI_HASH", artifact, f"canonical source hash is {observed}")


def validate_reasons(documents):
    artifact = "A04-safety-reason-registry.yaml"
    registry = documents[artifact]
    reasons = registry.get("reasons")
    require(isinstance(reasons, list) and len(reasons) == 37, "E_REASON_COUNT", artifact, "exactly 37 reasons are required")
    observed = [(item.get("bit"), item.get("name"), item.get("value")) for item in reasons if isinstance(item, dict)]
    expected = [(bit, name, 1 << bit) for bit, name in enumerate(REASONS)]
    require(observed == expected, "E_REASON_VALUE", artifact, "reason names, bits, and values must exactly match SafetyReason ABI v1")
    require(registry.get("defined_bits") == {"first": 0, "last": 36, "count": 37}, "E_REASON_RANGE", artifact, "defined_bits must be 0..36")
    require(registry.get("reserved_bits") == {"first": 37, "last": 63, "required_value": 0, "violation_maps_to": "INTERNAL_FAULT"}, "E_REASON_RANGE", artifact, "reserved bits must be zero and fail closed")


def require_false(document, keys, artifact):
    require(nested(document, keys, artifact) is False, "E_AUTHORITY", artifact, ".".join(keys) + " must be false")


def validate_closed_authority(value, artifact, location="<root>"):
    if isinstance(value, dict):
        for key, child in value.items():
            child_location = f"{location}.{key}"
            if key in AUTHORITY_FALSE_KEYS:
                require(child is False, "E_AUTHORITY", artifact, child_location + " must be false")
            validate_closed_authority(child, artifact, child_location)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            validate_closed_authority(child, artifact, f"{location}[{index}]")


def validate_authority(documents):
    for artifact, document in documents.items():
        if artifact.endswith((".yaml", ".yml")) and artifact != "manifest.yaml":
            validate_closed_authority(document, artifact)
    release = documents["A16-release-authority.yaml"]
    for key in ("software_release_candidate_authorized", "hardware_motion_authorized", "passenger_operation_authorized", "campus_operation_authorized", "team_selected"):
        require_false(release, ["release_scope", key], "A16-release-authority.yaml")
    require_false(release, ["blocked_profiles", "hardware_enabled", "allowed"], "A16-release-authority.yaml")
    for profile in ("sim", "replay", "hardware_shadow"):
        require_false(release, ["profile_constraints", profile, "real_motor_path_allowed"], "A16-release-authority.yaml")

    for artifact in ("collision-simulation-policy.yaml", "slope-simulation-policy.yaml"):
        policy = documents[artifact]
        require(policy.get("qualification") == "simulation_only", "E_SIMULATION_POLICY", artifact, "qualification must remain simulation_only")
        for key in ("hardware_motion_authorized", "passenger_operation_authorized", "transferable_to_hardware"):
            require_false(policy, ["authority", key], artifact)
    route = documents["route-safety-candidate.yaml"]
    require(route.get("provenance", {}).get("evidence_level") == "simulation_only", "E_SIMULATION_POLICY", "route-safety-candidate.yaml", "evidence_level must remain simulation_only")
    require_false(route, ["authority", "hardware_authorized"], "route-safety-candidate.yaml")
    require_false(route, ["authority", "passenger_authorized"], "route-safety-candidate.yaml")
    require(nested(route, ["authority", "simulation_only"], "route-safety-candidate.yaml") is True, "E_SIMULATION_POLICY", "route-safety-candidate.yaml", "simulation_only must be true")

    driver = documents["driver-unverified.yaml"]
    for key in ("verified", "hardware_motion_authorized", "passenger_operation_authorized"):
        require(driver.get(key) is False, "E_AUTHORITY", "driver-unverified.yaml", f"{key} must be false")
    require(driver.get("example_only") is False, "E_AUTHORITY", "driver-unverified.yaml", "example_only must be false")
    require(driver.get("adapter", {}).get("mode") == "disabled", "E_MOTOR_TOPIC", "driver-unverified.yaml", "adapter mode must be disabled")
    require(driver.get("command", {}).get("driver_topic") == "", "E_MOTOR_TOPIC", "driver-unverified.yaml", "unverified manifest must not name a real motor topic")
    fixture = documents["driver-verified-fixture.yaml"]
    require(fixture.get("example_only") is True, "E_AUTHORITY", "driver-verified-fixture.yaml", "verified-looking fixture must remain example_only")
    require(fixture.get("hardware_motion_authorized") is False and fixture.get("passenger_operation_authorized") is False, "E_AUTHORITY", "driver-verified-fixture.yaml", "fixture cannot authorize hardware or passengers")


def validate_hash_bindings(root, contract_dir, documents):
    for artifact in ("A00-review-lineage.md", "A01-adr-noetic-localization.md", "A02-interface-abi-v1.md"):
        text = (contract_dir / artifact).read_text(encoding="utf-8")
        require(f"source_plan_sha256: {PLAN_SHA256}" in text, "E_PLAN_HASH", artifact, "approved final-plan hash differs")
    for artifact in ("A16-release-authority.yaml", "A17-verification-matrix.yaml"):
        require(documents[artifact].get("source_plan_sha256") == PLAN_SHA256, "E_PLAN_HASH", artifact, "approved final-plan hash differs")
    provenance_hashes = {
        "plan_sha256": CONSENSUS_PLAN_SHA256,
        "architect_review_sha256": ARCHITECT_REVIEW_SHA256,
        "critic_review_sha256": CRITIC_REVIEW_SHA256,
    }
    for artifact in ("A12-target-nuc-method.yaml", "A13-simulator-fidelity.yaml", "A14-hazard-log.yaml"):
        provenance = documents[artifact].get("provenance", {})
        for key, expected in provenance_hashes.items():
            require(provenance.get(key) == expected, "E_PLAN_HASH", artifact, f"{key} differs from approved review lineage")
    evidence_plan = nested(documents["A15-evidence-inventory.yaml"], ["provenance", "approved_consensus_plan"], "A15-evidence-inventory.yaml")
    expected_evidence_hashes = {
        "planner_revision_sha256": CONSENSUS_PLAN_SHA256,
        "architect_review_sha256": ARCHITECT_REVIEW_SHA256,
        "critic_review_sha256": CRITIC_REVIEW_SHA256,
        "intent_reconciliation_sha256": INTENT_RECONCILIATION_SHA256,
    }
    for key, expected in expected_evidence_hashes.items():
        require(evidence_plan.get(key) == expected, "E_PLAN_HASH", "A15-evidence-inventory.yaml", f"{key} differs from approved review lineage")

    bindings = [
        ("collision-simulation-policy.yaml", "A07-collision-policy-schema.json"),
        ("slope-simulation-policy.yaml", "A08-slope-policy-schema.json"),
    ]
    for candidate, schema in bindings:
        expected = nested(documents[candidate], ["hashes", "schema_sha256", "value"], candidate)
        require(expected == sha256(contract_dir / schema), "E_SCHEMA_HASH", candidate, f"schema hash does not match {schema}")
        policy_hash = nested(documents[candidate], ["hashes", "policy_sha256", "value"], candidate)
        canonical = json.loads(json.dumps(documents[candidate]))
        del canonical["hashes"]["policy_sha256"]
        canonical_bytes = json.dumps(canonical, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        observed_policy_hash = hashlib.sha256(canonical_bytes).hexdigest()
        require(policy_hash == observed_policy_hash, "E_POLICY_HASH", candidate, f"canonical policy hash is {observed_policy_hash}")

    evidence = documents["A15-evidence-inventory.yaml"]
    source_bindings = [
        (["provenance", "repository_readme"], "README.md"),
        (["source_dataset", "metadata"], "data/hanyang_aegimun_loop/livox_rosbag_metadata.yaml"),
    ]
    source_bindings.extend((["committed_map_candidate", "artifacts", index], item["path"]) for index, item in enumerate(evidence["committed_map_candidate"]["artifacts"]))
    source_bindings.extend((["committed_route_candidate", "artifacts", index], item["path"]) for index, item in enumerate(evidence["committed_route_candidate"]["artifacts"]))
    for keys, relative_path in source_bindings:
        record = evidence
        for key in keys:
            record = record[key]
        path = root / relative_path
        require(path.is_file(), "E_SOURCE_MISSING", relative_path, "referenced source artifact is missing")
        require(record.get("sha256") == sha256(path), "E_SOURCE_HASH", relative_path, "referenced source hash differs")


def validate_a15(documents):
    artifact = "A15-evidence-inventory.yaml"
    bag = nested(documents[artifact], ["source_dataset", "full_bag"], artifact)
    require(bag.get("source_metadata_path") == "/home/mnjn/다운로드/livox/metadata.yaml", "E_EVIDENCE", artifact, "source metadata path differs from observed evidence")
    require(bag.get("metadata_declared_relative_file") == "livox_raw_20260707_191720_0.db3", "E_EVIDENCE", artifact, "metadata-declared filename differs")
    declared = bag.get("metadata_declared_file", {})
    require(
        declared.get("exists") is True
        and declared.get("file_type") == "symbolic_link"
        and declared.get("link_target") == "livox_raw_20260707_191720_0-001.db3"
        and declared.get("lstat_size_bytes") == 35
        and declared.get("stat_size_bytes") == 2813546496
        and declared.get("status") == "PRE_EXISTING_SOURCE_REPAIR_ALIAS"
        and declared.get("verifier_mutated_source") is False,
        "E_EVIDENCE",
        artifact,
        "metadata-declared path must record the observed immutable symlink alias",
    )
    actual = bag.get("actual_sqlite_segment", {})
    require(actual.get("filename") == "livox_raw_20260707_191720_0-001.db3", "E_EVIDENCE", artifact, "actual sqlite segment filename differs")
    require(actual.get("size_bytes") == 2813546496 and actual.get("message_rows") == 144484 and actual.get("topic_count") == 2, "E_EVIDENCE", artifact, "observed sqlite size/row/topic counts differ")
    expected_topics = [
        {"name": "/livox/lidar", "type": "livox_ros_driver2/msg/CustomMsg", "records": 6882},
        {"name": "/livox/imu", "type": "sensor_msgs/msg/Imu", "records": 137602},
    ]
    require(actual.get("storage_identifier") == "sqlite3" and actual.get("readable") is True, "E_EVIDENCE", artifact, "observed sqlite storage/readability differs")
    require(actual.get("topics") == expected_topics, "E_EVIDENCE", artifact, "observed sqlite topic inventory differs")
    require(actual.get("sha256", {}).get("status") == "REQUIRED_IN_GENERATED_STAGING_MANIFEST" and actual.get("sha256", {}).get("value") is None, "E_EVIDENCE", artifact, "large-file SHA must not be fabricated")
    mismatch = bag.get("filename_metadata_consistency", {})
    require(mismatch.get("status") == "PRE_EXISTING_ALIAS_EXPLICIT_STAGING_REQUIRED", "E_EVIDENCE", artifact, "source alias must require explicit staging")
    require(mismatch.get("silently_accepted") is False and mismatch.get("original_data_mutation_allowed") is False, "E_EVIDENCE", artifact, "alias must not be accepted or mutate source data")


def validate(root):
    if yaml is None:
        raise ContractError("E_DEPENDENCY", "PyYAML", "install PyYAML to parse required YAML contracts")
    contract_dir = root / "contracts" / "wp0"
    require(contract_dir.is_dir(), "E_CONTRACT_DIR", str(contract_dir), "contract directory is missing")
    for artifact in REQUIRED_ARTIFACTS + ["manifest.yaml"]:
        require((contract_dir / artifact).is_file(), "E_MISSING_ARTIFACT", artifact, "required WP0 contract is missing")
    documents = {}
    for artifact in REQUIRED_ARTIFACTS + ["manifest.yaml"]:
        path = contract_dir / artifact
        if path.suffix in (".yaml", ".yml"):
            documents[artifact] = load_yaml(path)
        elif path.suffix == ".json":
            documents[artifact] = load_json(path)
    validate_manifest(contract_dir, documents)
    validate_authority(documents)
    validate_schemas(contract_dir, documents)
    validate_abi(contract_dir)
    validate_reasons(documents)
    validate_hash_bindings(root, contract_dir, documents)
    validate_a15(documents)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.cwd(), help="repository root")
    args = parser.parse_args(argv)
    try:
        validate(args.root.resolve())
    except ContractError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print("WP0 contracts valid")
    return 0


if __name__ == "__main__":
    sys.exit(main())
