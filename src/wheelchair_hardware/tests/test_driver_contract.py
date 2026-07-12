#!/usr/bin/env python3
from copy import deepcopy
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


def test_hardware_and_passenger_authority_are_independent_at_adapter_boundary():
    manifest = authorized_manifest()
    manifest["passenger_operation_authorized"] = False
    authority = {
        "release_scope": {
            "hardware_motion_authorized": True,
            "passenger_operation_authorized": False,
        },
        "blocked_profiles": {"hardware_enabled": {"allowed": True}},
    }
    decision = SimpleNamespace(allowed=True, real_motor_path=True)
    runtime = {
        "platform_matches": True,
        "base_model_matches": True,
        "graph_valid": True,
    }
    preflight_decision = hardware_adapter._adapter_preflight(
        manifest, "hardware_enabled", runtime
    )
    assert preflight_decision.allowed
    assert preflight_decision.real_motor_path
    assert hardware_adapter._endpoint_authorized(
        manifest, authority, decision, "hardware_enabled", True, runtime
    )

    authority["release_scope"]["hardware_motion_authorized"] = False
    assert not hardware_adapter._endpoint_authorized(
        manifest, authority, decision, "hardware_enabled", True, runtime
    )


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
