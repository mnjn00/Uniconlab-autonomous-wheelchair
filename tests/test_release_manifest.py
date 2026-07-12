import copy
import importlib.util
import json
import os
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    path = ROOT / "scripts" / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


generate = load_script("generate_release_manifest")
# The verifier intentionally imports the generator as a sibling CLI module.
import sys
sys.path.insert(0, str(ROOT / "scripts"))
verify = load_script("verify_release_manifest")


def release_tree(tmp_path):
    files = {
        ".dockerignore": "build\n",
        "src/example/package.xml": "<package><name>example</name></package>\n",
        "src/example/CMakeLists.txt": "cmake_minimum_required(VERSION 3.0.2)\n",
        "src/example/setup.py": "from setuptools import setup\nsetup(name='example')\n",
        "src/wheelchair_interfaces/msg/SafetyState.msg": "bool stopped\n",
        "src/example/config/settings.yaml": "enabled: true\n",
        "src/example/launch/example.launch": "<launch/>\n",
        "src/wheelchair_description/urdf/wheelchair.urdf.xacro": "<robot name=\"wheelchair\"/>\n",
        "src/wheelchair_gazebo/worlds/example.world": "<sdf version=\"1.6\"/>\n",
        "contracts/wp0/contract.yaml": "authority: software_only\n",
        "data/site/map.pgm": "P2\n1 1\n255\n0\n",
        "data/site/map.yaml": "image: map.pgm\n",
        "data/site/map.metadata.json": "{}\n",
        "data/site/site.waypoints.yaml": "waypoints: []\n",
        "README.md": "# Operator guide\n",
        ".github/workflows/noetic-ci.yml": "name: noetic\n",
        "tools/noetic/Dockerfile": "FROM ubuntu:20.04@sha256:" + "0" * 64 + "\n",
        "tools/offline/requirements.lock": "pytest==6.2.5\n",
        "tests/test_qualification.py": "def test_qualification():\n    assert True\n",
        "reports/pytest.json": json.dumps({
            "artifactType": "algorithm-adversarial-test-report",
            "schemaVersion": 1,
            "status": "PASS",
            "claimTag": "SIMULATION_ONLY",
            "surface": "simulation",
            "hardwareMotionAuthorized": False,
            "passengerOperationAuthorized": False,
            "source_revision": "a" * 40,
            "result": {"passed": True, "summary": {"total": 1, "failed": 0}},
        }) + "\n",
    }
    for relative in generate.REQUIRED_RUNTIME_ENTRYPOINTS:
        files[relative] = "#!/usr/bin/env python3\nprint('safe')\n"
    for relative, content in files.items():
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    for relative in generate.REQUIRED_RUNTIME_ENTRYPOINTS:
        (tmp_path / relative).chmod(0o755)
    source = {"kind": "git_commit", "revision": "a" * 40, "worktree_clean": True}
    return tmp_path / "reports/pytest.json", source


def make_manifest(tmp_path):
    report, source = release_tree(tmp_path)
    manifest = generate.generate_manifest(tmp_path, [report], "parent-unarmed", source=source)
    output = tmp_path / "release-manifest.json"
    generate.atomic_write(output, manifest)
    return output, manifest


def write_report(path, document):
    if isinstance(document, str):
        path.write_text(document, encoding="utf-8")
    else:
        path.write_text(json.dumps(document) + "\n", encoding="utf-8")


def test_manifest_is_deterministic_and_verifiable(tmp_path):
    report, source = release_tree(tmp_path)
    first = generate.generate_manifest(tmp_path, [report], "parent-unarmed", source=source)
    second = generate.generate_manifest(tmp_path, [report], "parent-unarmed", source=source)
    assert first == second
    output = tmp_path / "release-manifest.json"
    generate.atomic_write(output, first)
    assert verify.verify_manifest(output, tmp_path) == first
    third = generate.generate_manifest(tmp_path, [report], "parent-unarmed", source=source)
    assert third == first
def test_excluded_runtime_symlink_is_ignored_by_generation_and_verification(tmp_path):
    report, source = release_tree(tmp_path)
    runtime_link = tmp_path / "artifacts/qa/roslog-startup-final/latest"
    runtime_link.parent.mkdir(parents=True)
    runtime_link.symlink_to(tmp_path.parent / "outside-release-root")

    manifest = generate.generate_manifest(
        tmp_path, [report], "parent-unarmed", source=source)
    output = tmp_path / "release-manifest.json"
    generate.atomic_write(output, manifest)

    assert verify.verify_manifest(output, tmp_path) == manifest


def test_nonexcluded_escaping_symlink_is_rejected_by_generation_and_verification(tmp_path):
    report, source = release_tree(tmp_path)
    escaping = tmp_path / "docs/escaping-link"
    escaping.parent.mkdir()
    escaping.symlink_to(tmp_path.parent / "outside-release-root")

    with pytest.raises(generate.ManifestError, match="symlink escapes release root"):
        generate.generate_manifest(tmp_path, [report], "parent-unarmed", source=source)

    escaping.unlink()
    output, _ = make_manifest(tmp_path)
    escaping.symlink_to(tmp_path.parent / "outside-release-root")
    with pytest.raises(verify.ManifestError, match="symlink escapes release root"):
        verify.verify_manifest(output, tmp_path)


def test_explicit_report_symlinks_are_rejected(tmp_path):
    report, source = release_tree(tmp_path)
    report.unlink()
    report.symlink_to(tmp_path / "reports/replacement.json")

    with pytest.raises(generate.ManifestError, match="inventory entry is a symlink"):
        generate.generate_manifest(tmp_path, [report], "parent-unarmed", source=source)

    report.unlink()
    write_report(report, clean_simulation_report())
    manifest = generate.generate_manifest(
        tmp_path, [report], "parent-unarmed", source=source)
    output = tmp_path / "release-manifest.json"
    generate.atomic_write(output, manifest)
    replacement = tmp_path / "reports/replacement.json"
    write_report(replacement, clean_simulation_report())
    report.unlink()
    report.symlink_to(replacement)

    with pytest.raises(verify.ManifestError, match="test report is a symlink"):
        verify.verify_manifest(output, tmp_path)



def test_manifest_includes_representative_clean_target_assets(tmp_path):
    _, manifest = make_manifest(tmp_path)
    by_category = {
        category: {entry["path"] for entry in section["files"]}
        for category, section in manifest["hashes"].items()
    }
    assert "src/example/CMakeLists.txt" in by_category["package_metadata"]
    assert "src/wheelchair_interfaces/msg/SafetyState.msg" in by_category["interfaces"]
    assert "src/example/config/settings.yaml" in by_category["configuration"]
    assert "data/site/map.yaml" in by_category["maps"]
    assert "data/site/site.waypoints.yaml" in by_category["routes"]
    assert "src/example/launch/example.launch" in by_category["launch_configuration"]
    assert "src/wheelchair_description/urdf/wheelchair.urdf.xacro" in by_category["robot_assets"]
    assert "README.md" in by_category["operator_docs"]
    assert ".github/workflows/noetic-ci.yml" in by_category["ci_tools"]
    assert set(generate.REQUIRED_RUNTIME_ENTRYPOINTS) <= by_category["python_runtime"]
    assert ".dockerignore" in by_category["source_build_metadata"]


@pytest.mark.parametrize("entrypoint", generate.REQUIRED_RUNTIME_ENTRYPOINTS)
def test_missing_required_runtime_entrypoint_is_rejected(tmp_path, entrypoint):
    output, _ = make_manifest(tmp_path)
    (tmp_path / entrypoint).unlink()
    with pytest.raises(verify.VerificationError, match="runtime entrypoint|required hash category"):
        verify.verify_manifest(output, tmp_path)


@pytest.mark.parametrize("entrypoint", generate.REQUIRED_RUNTIME_ENTRYPOINTS)
def test_tampered_required_runtime_entrypoint_is_rejected(tmp_path, entrypoint):
    output, _ = make_manifest(tmp_path)
    (tmp_path / entrypoint).write_text("#!/usr/bin/env python3\nprint('tampered')\n",
                                       encoding="utf-8")
    with pytest.raises(verify.VerificationError, match="hash mismatch"):
        verify.verify_manifest(output, tmp_path)


def test_required_runtime_executable_mode_tamper_is_rejected(tmp_path):
    output, _ = make_manifest(tmp_path)
    entrypoint = tmp_path / generate.REQUIRED_RUNTIME_ENTRYPOINTS[0]
    entrypoint.chmod(0o644)
    with pytest.raises(verify.VerificationError, match="not executable|executable mode"):
        verify.verify_manifest(output, tmp_path)


def test_new_unbound_deployable_file_is_rejected(tmp_path):
    output, _ = make_manifest(tmp_path)
    (tmp_path / "src/example/config/unbound.yaml").write_text("unsafe: true\n", encoding="utf-8")
    with pytest.raises(verify.VerificationError, match="hash scope differs"):
        verify.verify_manifest(output, tmp_path)


def test_generation_rejects_unknown_regular_file(tmp_path):
    report, source = release_tree(tmp_path)
    (tmp_path / "unclassified.future").write_text("must not be omitted\n", encoding="utf-8")
    with pytest.raises(generate.ManifestError, match="unclassified regular file: unclassified.future"):
        generate.generate_manifest(tmp_path, [report], "parent-unarmed", source=source)


def test_verifier_independently_rejects_unknown_regular_file(tmp_path):
    output, _ = make_manifest(tmp_path)
    (tmp_path / "unclassified.future").write_text("must not be omitted\n", encoding="utf-8")
    with pytest.raises(verify.VerificationError, match="unclassified regular file"):
        verify.verify_manifest(output, tmp_path)


def test_clean_junit_report_is_accepted(tmp_path):
    report, source = release_tree(tmp_path)
    xml_report = tmp_path / "reports/junit.xml"
    report.unlink()
    write_report(xml_report, '<testsuite tests="2" failures="0" errors="0" skipped="0"/>')
    manifest = generate.generate_manifest(
        tmp_path, [xml_report], "parent-unarmed", source=source)
    output = tmp_path / "release-manifest.json"
    generate.atomic_write(output, manifest)
    assert verify.verify_manifest(output, tmp_path) == manifest


@pytest.mark.parametrize("field", ["failures", "errors", "skipped"])
def test_unclean_junit_report_is_rejected(tmp_path, field):
    report, source = release_tree(tmp_path)
    report.unlink()
    xml_report = tmp_path / "reports/junit.xml"
    counts = {"tests": 2, "failures": 0, "errors": 0, "skipped": 0}
    counts[field] = 1
    write_report(xml_report, (
        '<testsuite tests="{tests}" failures="{failures}" errors="{errors}" '
        'skipped="{skipped}"/>').format(**counts))
    with pytest.raises(generate.ManifestError, match="not a clean run"):
        generate.generate_manifest(
            tmp_path, [xml_report], "parent-unarmed", source=source)


def clean_simulation_report():
    return {
        "artifactType": "algorithm-adversarial-test-report",
        "schemaVersion": 1,
        "status": "PASS",
        "claimTag": "SIMULATION_ONLY",
        "surface": "simulation",
        "hardwareMotionAuthorized": False,
        "passengerOperationAuthorized": False,
        "source_revision": "a" * 40,
        "result": {"passed": True, "summary": {"total": 4, "failed": 0}},
    }


def test_clean_simulation_json_is_accepted(tmp_path):
    report, source = release_tree(tmp_path)
    write_report(report, clean_simulation_report())
    manifest = generate.generate_manifest(tmp_path, [report], "parent-unarmed", source=source)
    output = tmp_path / "release-manifest.json"
    generate.atomic_write(output, manifest)
    assert verify.verify_manifest(output, tmp_path) == manifest


@pytest.mark.parametrize(
    "mutation",
    [
        lambda value: value.update(status="PLATFORM_UNAVAILABLE"),
        lambda value: value.update(status="FAIL"),
        lambda value: value["result"].update(passed=False),
        lambda value: value.update(surface="unit"),
        lambda value: value.pop("artifactType"),
        lambda value: value.pop("schemaVersion"),
        lambda value: value.update(hardwareMotionAuthorized=True),
        lambda value: value.update(passengerOperationAuthorized=True),
    ],
)
def test_unavailable_failed_promoted_or_malformed_json_is_rejected(tmp_path, mutation):
    report, source = release_tree(tmp_path)
    document = clean_simulation_report()
    mutation(document)
    write_report(report, document)
    with pytest.raises(generate.ManifestError):
        generate.generate_manifest(tmp_path, [report], "parent-unarmed", source=source)


def test_semantically_tampered_report_is_rejected_even_when_manifest_is_rebound(tmp_path):
    output, manifest = make_manifest(tmp_path)
    report = tmp_path / "reports/pytest.json"
    document = clean_simulation_report()
    document["result"]["passed"] = False
    write_report(report, document)
    entry = manifest["hashes"]["qualification_evidence"]["files"][0]
    entry["sha256"] = generate.file_hash(report)
    manifest["test_reports"][0]["sha256"] = entry["sha256"]
    entries = manifest["hashes"]["qualification_evidence"]["files"]
    manifest["hashes"]["qualification_evidence"]["digest"] = generate.canonical_hash(entries)
    manifest.pop("release_binding_sha256")
    manifest["release_binding_sha256"] = generate.canonical_hash(manifest)
    generate.atomic_write(output, manifest)
    with pytest.raises(verify.VerificationError, match="failed result"):
        verify.verify_manifest(output, tmp_path)


def test_tamper_is_rejected(tmp_path):
    output, _ = make_manifest(tmp_path)
    (tmp_path / "src/example/config/settings.yaml").write_text("enabled: false\n", encoding="utf-8")
    with pytest.raises(verify.VerificationError, match="hash mismatch"):
        verify.verify_manifest(output, tmp_path)


def test_missing_report_is_rejected(tmp_path):
    output, _ = make_manifest(tmp_path)
    (tmp_path / "reports/pytest.json").unlink()
    with pytest.raises(verify.VerificationError, match="missing test report|missing or escapes"):
        verify.verify_manifest(output, tmp_path)


def test_authority_escalation_is_rejected_even_with_rebound_manifest(tmp_path):
    output, manifest = make_manifest(tmp_path)
    escalated = copy.deepcopy(manifest)
    escalated["authority"]["hardware_motion_authorized"] = True
    escalated.pop("release_binding_sha256")
    escalated["release_binding_sha256"] = generate.canonical_hash(escalated)
    generate.atomic_write(output, escalated)
    with pytest.raises(verify.VerificationError, match="authority escalation"):
        verify.verify_manifest(output, tmp_path)


def test_atomic_write_preserves_previous_manifest_on_replace_failure(tmp_path, monkeypatch):
    output = tmp_path / "release-manifest.json"
    output.write_text("previous\n", encoding="utf-8")

    def fail_replace(_source, _target):
        raise OSError("simulated atomic replacement failure")

    monkeypatch.setattr(os, "replace", fail_replace)
    with pytest.raises(OSError, match="simulated"):
        generate.atomic_write(output, {"new": True})
    assert output.read_text(encoding="utf-8") == "previous\n"
    assert list(tmp_path.glob(".release-manifest.json.*")) == []


def test_rollback_to_armed_state_is_rejected(tmp_path):
    output, manifest = make_manifest(tmp_path)
    armed = copy.deepcopy(manifest)
    armed["rollback"]["parent_state"] = "armed"
    armed.pop("release_binding_sha256")
    armed["release_binding_sha256"] = generate.canonical_hash(armed)
    generate.atomic_write(output, armed)
    with pytest.raises(verify.VerificationError, match="armed state"):
        verify.verify_manifest(output, tmp_path)
