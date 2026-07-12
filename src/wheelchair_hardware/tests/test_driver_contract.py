#!/usr/bin/env python3
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "src" / "wheelchair_hardware"
SCHEMA = ROOT / "contracts" / "wp0" / "A11-driver-contract-schema.json"
sys.path.insert(0, str(PACKAGE / "scripts"))

from driver_contract import DriverContractError, load_manifest, preflight, validate_manifest
import hardware_adapter


class FakeDuration:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


class FakeTime:
    def __init__(self, seconds):
        self.seconds = seconds

    def __sub__(self, other):
        return FakeDuration(self.seconds - other.seconds)


def codes(manifest):
    return {issue.code for issue in validate_manifest(manifest, SCHEMA)}


def fixture():
    return load_manifest(ROOT / "contracts" / "wp0" / "driver-verified-fixture.yaml")


def authorized_manifest():
    manifest = fixture()
    manifest["contract_id"] = "synthetic-authorized-test-only"
    manifest["example_only"] = False
    manifest["hardware_motion_authorized"] = True
    manifest["passenger_operation_authorized"] = True
    manifest["approval"]["campus_approval_id"] = "SYNTHETIC-TEST-ONLY"
    return manifest


def test_installed_default_exactly_matches_wp0_and_has_no_motor_path():
    installed = PACKAGE / "config" / "driver-unverified.yaml"
    source = ROOT / "contracts" / "wp0" / "driver-unverified.yaml"
    assert installed.read_bytes() == source.read_bytes()
    manifest = load_manifest(installed)
    assert validate_manifest(manifest, SCHEMA) == ()
    assert manifest["command"]["driver_topic"] == ""
    for profile in ("sim", "replay", "hardware_shadow"):
        decision = preflight(manifest, profile, schema_path=SCHEMA)
        assert decision.allowed
        assert not decision.deployable
        assert not decision.real_motor_path
        assert decision.driver_topic == ""


def test_load_rejects_malformed_or_non_mapping_yaml(tmp_path):
    malformed = tmp_path / "malformed.yaml"
    malformed.write_text("command: [\n")
    with pytest.raises(DriverContractError) as error:
        load_manifest(malformed)
    assert error.value.code == "E_FORMAT"

    scalar = tmp_path / "scalar.yaml"
    scalar.write_text("disabled\n")
    with pytest.raises(DriverContractError) as error:
        load_manifest(scalar)
    assert error.value.code == "E_FORMAT"


def test_absent_and_extra_fields_have_stable_codes():
    manifest = load_manifest(PACKAGE / "config" / "driver-unverified.yaml")
    del manifest["mode"]
    assert "E_REQUIRED" in codes(manifest)

    manifest = load_manifest(PACKAGE / "config" / "driver-unverified.yaml")
    manifest["fallback_driver_topic"] = "/motors"
    assert "E_UNKNOWN_FIELD" in codes(manifest)

    manifest = load_manifest(PACKAGE / "config" / "driver-unverified.yaml")
    manifest["command"] = "not-a-command-mapping"
    assert "E_FORMAT" in codes(manifest)


def test_fake_verified_fixture_is_valid_but_never_deployable():
    manifest = fixture()
    assert validate_manifest(manifest, SCHEMA) == ()
    decision = preflight(
        manifest,
        "hardware_enabled",
        {"platform_matches": True, "base_model_matches": True, "graph_valid": True},
        SCHEMA,
    )
    assert not decision.allowed
    assert not decision.real_motor_path
    assert "E_EXAMPLE_ONLY" in decision.error_codes
    assert "E_AUTHORIZATION_FALSE" in decision.error_codes


def test_unverified_manifest_cannot_name_or_advertise_real_topic():
    manifest = load_manifest(PACKAGE / "config" / "driver-unverified.yaml")
    manifest["command"]["driver_topic"] = "/base_controller/cmd_vel"
    assert "E_UNVERIFIED" in codes(manifest)
    decision = preflight(manifest, "hardware_enabled", schema_path=SCHEMA)
    assert not decision.allowed
    assert decision.driver_topic == ""
    assert not decision.real_motor_path


@pytest.mark.parametrize(
    "section,field,expected",
    [
        ("command", "timeout_s", "E_REQUIRED"),
        ("manual_override", "status_topic", "E_REQUIRED"),
        ("estop", "status_topic", "E_REQUIRED"),
        ("mode", "auto_value", "E_REQUIRED"),
        ("mode", "manual_value", "E_REQUIRED"),
        ("odometry", "topic", "E_REQUIRED"),
    ],
)
def test_required_timeout_override_estop_auto_manual_and_odom(section, field, expected):
    manifest = fixture()
    del manifest[section][field]
    assert expected in codes(manifest)


def test_explicit_direct_and_translated_adapter_modes():
    direct = fixture()
    assert validate_manifest(direct, SCHEMA) == ()

    translated = fixture()
    translated["adapter"] = {
        "mode": "translated",
        "ros_package": "synthetic_adapter",
        "executable": "synthetic_adapter_node",
        "translation_spec_sha256": "3" * 64,
    }
    translated["command"]["driver_topic"] = "/synthetic/driver_command"
    translated["command"]["message_type"] = "std_msgs/Float64MultiArray"
    translated["command"]["message_md5"] = "4" * 32
    assert validate_manifest(translated, SCHEMA) == ()

    translated["adapter"]["translation_spec_sha256"] = ""
    assert "E_TRANSLATION_SPEC" in codes(translated)
    translated = authorized_manifest()
    translated["adapter"] = {
        "mode": "translated",
        "ros_package": "synthetic_adapter",
        "executable": "synthetic_adapter_node",
        "translation_spec_sha256": "3" * 64,
    }
    translated["command"].update({
        "driver_topic": "/synthetic/driver_command",
        "message_type": "std_msgs/Float64MultiArray",
        "message_md5": "4" * 32,
    })
    evidence = {"platform_matches": True, "base_model_matches": True, "graph_valid": True}
    decision = preflight(translated, "hardware_enabled", evidence, SCHEMA)
    assert validate_manifest(translated, SCHEMA) == ()
    assert "E_ADAPTER_MODE" in decision.error_codes
    assert not decision.deployable
    assert not decision.real_motor_path
    assert not hardware_adapter._endpoint_authorized(
        translated,
        {"release_scope": {"hardware_motion_authorized": True,
                           "passenger_operation_authorized": True},
         "blocked_profiles": {"hardware_enabled": {"allowed": True}}},
        decision,
        "hardware_enabled",
        True,
        dict(evidence, receipt_verified=True),
    )


def test_nonfinite_reversed_and_nonspanning_values_fail_closed():
    for path, value, expected in (
        (("command", "publish_rate_hz"), float("nan"), "E_RATE"),
        (("command", "timeout_s"), float("inf"), "E_TIMEOUT"),
        (("command", "linear", "minimum"), 0.01, "E_AXIS"),
        (("mode", "stale_timeout_s"), float("nan"), "E_MODE_STATUS"),
    ):
        manifest = fixture()
        target = manifest
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = value
        assert expected in codes(manifest)


def test_navigation_bypass_is_rejected_anywhere_in_boundary_fields():
    manifest = fixture()
    manifest["mode"]["request_name"] = "/cmd_vel_nav"
    assert "E_NAV_BYPASS" in codes(manifest)


def test_synthetic_authorized_manifest_needs_every_runtime_evidence_flag():
    manifest = authorized_manifest()
    assert validate_manifest(manifest, SCHEMA) == ()

    required = {"platform_matches": True, "base_model_matches": True, "graph_valid": True}
    for omitted in tuple(required):
        evidence = deepcopy(required)
        del evidence[omitted]
        decision = preflight(manifest, "hardware_enabled", evidence, SCHEMA)
        assert not decision.allowed
        assert not decision.real_motor_path

    decision = preflight(manifest, "hardware_enabled", required, SCHEMA)
    assert decision.allowed
    assert decision.deployable
    assert decision.real_motor_path
    assert decision.driver_topic == "/cmd_vel_safe"
    assert decision.errors == ()


def test_unknown_profile_is_rejected_without_exposing_topic():
    decision = preflight(authorized_manifest(), "hardware", schema_path=SCHEMA)
    assert not decision.allowed
    assert not decision.real_motor_path
    assert decision.driver_topic == ""
    assert "E_FORMAT" in decision.error_codes


def test_adapter_uses_canonical_fail_closed_evidence_topics_at_twenty_hz():
    assert hardware_adapter._CANONICAL_DRIVER_TOPIC == "/hardware/driver_status"
    assert hardware_adapter._CANONICAL_MODE_TOPIC == "/safety/mode"
    assert hardware_adapter._CANONICAL_DRIVER_SIGNAL_TOPIC == "/safety/driver"
    assert hardware_adapter._STATUS_PERIOD_S <= 1.0 / 20.0

    manifest = fixture()
    samples = {
        "mode": {"value": "auto", "stamp": FakeTime(10.0)},
        "override": {"value": False, "stamp": FakeTime(10.0)},
        "estop": {"value": False, "stamp": FakeTime(10.0)},
    }
    shadow = hardware_adapter._evidence_decision(
        manifest, False, samples, FakeTime(10.01)
    )
    assert not shadow[0]
    assert shadow[-1] & hardware_adapter._DRIVER_REASON
    assert shadow[-1] & hardware_adapter._INPUT_UNKNOWN_REASON


def test_either_false_authority_flag_rejects_adapter_boundary():
    manifest = authorized_manifest()
    authority = {
        "release_scope": {
            "hardware_motion_authorized": True,
            "passenger_operation_authorized": True,
        },
        "blocked_profiles": {"hardware_enabled": {"allowed": True}},
    }
    decision = SimpleNamespace(allowed=True, real_motor_path=True)
    runtime = {
        "platform_matches": True,
        "base_model_matches": True,
        "graph_valid": True,
        "receipt_verified": True,
    }
    assert hardware_adapter._endpoint_authorized(
        manifest, authority, decision, "hardware_enabled", True, runtime
    )

    manifest["passenger_operation_authorized"] = False
    assert not hardware_adapter._adapter_preflight(
        manifest, "hardware_enabled", runtime
    ).allowed
    assert not hardware_adapter._endpoint_authorized(
        manifest, authority, decision, "hardware_enabled", True, runtime
    )
    manifest["passenger_operation_authorized"] = True
    authority["release_scope"]["hardware_motion_authorized"] = False
    assert not hardware_adapter._endpoint_authorized(
        manifest, authority, decision, "hardware_enabled", True, runtime
    )
    authority["release_scope"]["hardware_motion_authorized"] = True
    runtime["receipt_verified"] = False
    assert not hardware_adapter._endpoint_authorized(
        manifest, authority, decision, "hardware_enabled", True, runtime
    )


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def _seal_release(release):
    unsigned = {
        key: value for key, value in release.items()
        if key not in {"release_binding_sha256", "release_signature_hmac_sha256"}
    }
    release["release_binding_sha256"] = hardware_adapter._canonical_hash(unsigned)
    release["release_signature_hmac_sha256"] = "a" * 64

def _drop_inventoried_report(release):
    category = release["hashes"]["qualification_evidence"]
    missing_path = release["test_reports"][0]["path"]
    category["files"] = [
        entry for entry in category["files"] if entry["path"] != missing_path
    ]
    category["digest"] = hardware_adapter._canonical_hash(category["files"])


def _v2_release_fixture(tmp_path):
    report_hashes = {
        path: hashlib.sha256(path.encode("utf-8")).hexdigest()
        for path in hardware_adapter._RELEASE_GATE_REPORTS.values()
    }
    hashes = {}
    for category in hardware_adapter._RELEASE_INVENTORY_CATEGORIES:
        files = [{
            "path": "inventory/{}.json".format(category),
            "sha256": hashlib.sha256(category.encode("utf-8")).hexdigest(),
            "executable": False,
        }]
        if category == "qualification_evidence":
            files.extend({
                "path": path, "sha256": digest, "executable": False,
            } for path, digest in report_hashes.items())
        files.sort(key=lambda entry: entry["path"])
        hashes[category] = {
            "files": files,
            "digest": hardware_adapter._canonical_hash(files),
        }
    source = {"kind": "git_commit", "revision": "a" * 40, "worktree_clean": True}
    digests = {name: hashes[name]["digest"] for name in hashes}
    release_input = hardware_adapter._canonical_hash({
        name: digests[name] for name in hardware_adapter._RELEASE_INVENTORY_CATEGORIES
        if name != "qualification_evidence"
    })
    bindings = {
        "sourceRevision": source["revision"],
        "configurationDigest": digests["configuration"],
        "releaseInputDigest": release_input,
    }
    bindings["bundleDigest"] = hardware_adapter._canonical_hash({
        "source_revision": source["revision"],
        "configuration_digest": bindings["configurationDigest"],
        "release_input_digest": release_input,
    })
    release = {
        "schema": "wheelchair-noetic-release-manifest/v2",
        "source": source,
        "hashes": hashes,
        "gate_matrix": {
            "requiredGateIds": sorted(hardware_adapter._RELEASE_GATE_REPORTS),
            "passedGateIds": sorted(hardware_adapter._RELEASE_GATE_REPORTS),
            "releaseBindings": bindings,
        },
        "authority": deepcopy(hardware_adapter._RELEASE_AUTHORITY),
        "qualification": deepcopy(hardware_adapter._RELEASE_QUALIFICATION),
        "test_reports": [
            {"path": path, "sha256": report_hashes[path], "executable": False}
            for path in sorted(report_hashes)
        ],
        "residual_blockers": list(hardware_adapter._RELEASE_RESIDUAL_BLOCKERS),
        "rollback": {
            "parentReleaseBindingSha256": "b" * 64,
            "parentManifestSha256": "c" * 64,
            "parentManifestPath": "release/parent.json",
            "parentInventoryDigest": "d" * 64,
            "restartReceipt": {
                "path": "evidence/release/restart.json",
                "sha256": "e" * 64,
                "parentReleaseBindingSha256": "b" * 64,
                "parentInventoryDigest": "d" * 64,
            },
        },
    }
    _seal_release(release)
    path = tmp_path / "release-manifest.json"
    path.write_text(json.dumps(release, sort_keys=True))
    return path, release


def test_v2_release_manifest_requires_signed_software_only_structure(tmp_path):
    path, release = _v2_release_fixture(tmp_path)
    parsed, inventory = hardware_adapter._release_manifest(path)
    assert parsed == release
    assert not parsed["authority"]["hardware_motion_authorized"]
    assert inventory[release["test_reports"][0]["path"]] == release["test_reports"][0]["sha256"]


@pytest.mark.parametrize(
    "mutate,reseal",
    [
        (lambda release: release.update(schema="wheelchair-noetic-release-manifest/v1"), False),
        (lambda release: release.pop("release_signature_hmac_sha256"), False),
        (lambda release: release.update(release_signature_hmac_sha256="not-a-signature"), False),
        (lambda release: release.update(release_binding_sha256="0" * 64), False),
        (lambda release: release.update(source={
            "kind": "worktree", "revision": "worktree:dirty", "worktree_clean": False,
        }), True),
        (lambda release: release["authority"].update(hardware_motion_authorized=True), True),
        (lambda release: release.update(residual_blockers=["hardware_motion_unqualified"]), True),
        (_drop_inventoried_report, True),
    ],
)
def test_v2_release_manifest_rejects_stale_tampered_or_authority_promoted_cases(
        tmp_path, mutate, reseal):
    path, release = _v2_release_fixture(tmp_path)
    mutate(release)
    if reseal:
        _seal_release(release)
    path.write_text(json.dumps(release, sort_keys=True))
    with pytest.raises(DriverContractError):
        hardware_adapter._release_manifest(path)


def test_runtime_evidence_rejects_empty_bundle_and_asserted_booleans(tmp_path):
    root = tmp_path / "bundle"
    root.mkdir()
    driver = root / "driver.yaml"
    authority = root / "authority.yaml"
    release = root / "release-manifest.json"
    driver.write_bytes((ROOT / "contracts/wp0/driver-verified-fixture.yaml").read_bytes())
    authority.write_bytes((ROOT / "contracts/wp0/A16-release-authority.yaml").read_bytes())
    release.write_text("{}")
    receipt = root / "runtime-evidence.json"
    receipt.write_text(json.dumps({
        "schema_version": 1,
        "status": "verified",
        "driver_manifest_path": driver.name,
        "driver_manifest_sha256": _sha(driver),
        "release_authority_path": authority.name,
        "release_authority_sha256": _sha(authority),
        "bundle_manifest_path": release.name,
        "bundle_manifest_sha256": _sha(release),
        "platform_matches": True,
        "base_model_matches": True,
        "graph_valid": True,
    }, sort_keys=True))
    with pytest.raises(DriverContractError):
        hardware_adapter._load_runtime_evidence(
            root, receipt.name, _sha(receipt), driver, authority
        )


def test_raw_platform_and_graph_receipts_are_identity_bound():
    manifest = fixture()
    platform = dict(manifest["platform"])
    platform.update({
        "schema_version": 1,
        "artifact_type": "wheelchair-platform-measurement/v1",
    })
    hardware_adapter._measured_platform(platform, manifest)
    platform["serial"] = "forged"
    with pytest.raises(DriverContractError):
        hardware_adapter._measured_platform(platform, manifest)

    graph = dict(manifest["evidence"])
    graph.update({
        "schema_version": 1,
        "artifact_type": "wheelchair-graph-measurement/v1",
    })
    hardware_adapter._measured_graph(graph, manifest)
    graph["graph_snapshot_sha256"] = "0" * 64
    with pytest.raises(DriverContractError):
        hardware_adapter._measured_graph(graph, manifest)


def _twist(linear_x=0.0, angular_z=0.0, **axes):
    return SimpleNamespace(
        linear=SimpleNamespace(
            x=linear_x, y=axes.get("linear_y", 0.0), z=axes.get("linear_z", 0.0)
        ),
        angular=SimpleNamespace(
            x=axes.get("angular_x", 0.0), y=axes.get("angular_y", 0.0), z=angular_z
        ),
    )


def test_final_twist_contract_rejects_nonfinite_axes_and_bounds():
    command = authorized_manifest()["command"]
    assert hardware_adapter._twist_contract_error(_twist(), command) is None
    assert hardware_adapter._twist_contract_error(
        _twist(linear_x=float("nan")), command
    ) == "nonfinite"
    assert hardware_adapter._twist_contract_error(
        _twist(linear_y=0.01), command
    ) == "unsupported_axis"
    assert hardware_adapter._twist_contract_error(
        _twist(linear_x=command["linear"]["maximum"] + 0.01), command
    ) == "linear_bounds"
    assert hardware_adapter._twist_contract_error(
        _twist(angular_z=command["angular"]["minimum"] - 0.01), command
    ) == "angular_bounds"


def test_generic_adapter_never_publishes_to_direct_input():
    source = (PACKAGE / "scripts/hardware_adapter.py").read_text()
    assert "rospy.Publisher(driver_topic" not in source
    assert "if not enabled:" in source
    assert "status.measured_linear_mps = -1.0" in source
    assert "status.measured_angular_rps = -1.0" in source


def test_enabled_clear_requires_fresh_actual_mode_override_and_estop():
    manifest = fixture()
    samples = {
        "mode": {"value": "auto", "stamp": FakeTime(10.0)},
        "override": {"value": False, "stamp": FakeTime(10.0)},
        "estop": {"value": False, "stamp": FakeTime(10.0)},
    }
    clear = hardware_adapter._evidence_decision(
        manifest, True, samples, FakeTime(10.01)
    )
    assert clear[0]
    assert clear[-1] == 0

    samples["estop"]["value"] = True
    asserted = hardware_adapter._evidence_decision(
        manifest, True, samples, FakeTime(10.01)
    )
    assert not asserted[0]
    assert asserted[-1] & hardware_adapter._ESTOP_REASON

    samples["estop"]["value"] = False
    stale = hardware_adapter._evidence_decision(
        manifest, True, samples, FakeTime(10.16)
    )
    assert not stale[0]
    assert stale[-1] & hardware_adapter._SENSOR_STALE_REASON
