import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    path = ROOT / "scripts" / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sys.path.insert(0, str(ROOT / "scripts"))
generate = load_script("generate_release_manifest")
verify = load_script("verify_release_manifest")
SOURCE = {"kind": "git_commit", "revision": "a" * 40, "worktree_clean": True}


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")


def release_tree(tmp_path):
    files = {
        ".dockerignore": "build\n",
        "src/example/package.xml": "<package><name>example</name></package>\n",
        "src/example/CMakeLists.txt": "cmake_minimum_required(VERSION 3.0.2)\n",
        "src/example/setup.py": "from setuptools import setup\nsetup(name='example')\n",
        "src/wheelchair_interfaces/msg/SafetyState.msg": "bool stopped\n",
        "src/example/config/settings.yaml": "enabled: true\n",
        "src/example/launch/example.launch": "<launch/>\n",
        "src/wheelchair_description/urdf/wheelchair.urdf.xacro": "<robot name='wheelchair'/>\n",
        "contracts/wp0/contract.yaml": "authority: software_only\n",
        "data/site/map.pgm": "P2\n1 1\n255\n0\n",
        "data/site/site.waypoints.yaml": "waypoints: []\n",
        "README.md": "# Operator guide\n",
        ".github/workflows/noetic-ci.yml": "name: noetic\n",
        "tests/test_qualification.py": "def test_qualification():\n    assert True\n",
    }
    for entrypoint in generate.REQUIRED_RUNTIME_ENTRYPOINTS:
        files[entrypoint] = "#!/usr/bin/env python3\nprint('safe')\n"
    for relative, content in files.items():
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    for entrypoint in generate.REQUIRED_RUNTIME_ENTRYPOINTS:
        (tmp_path / entrypoint).chmod(0o755)
    for relative in generate.REQUIRED_GATES.values():
        write_json(tmp_path / relative, {})
    return [tmp_path / path for path in sorted(generate.REQUIRED_GATES.values())]


def bindings_for(root, reports):
    del reports
    return generate.prepare_bindings(root, source=SOURCE)


def gate_report(gate_id, bindings):
    return {
        "artifactType": "wheelchair-ac-gate-report",
        "schemaVersion": 1,
        "gateId": gate_id,
        "status": "PASS",
        "claimTag": "SOFTWARE_ONLY",
        "hardwareMotionAuthorized": False,
        "passengerOperationAuthorized": False,
        **bindings,
        "result": {"passed": True, "cases": 1},
    }


def rollback_binding():
    inventory = {
        kind: [{"path": kind + "/item", "sha256": (str(index) * 64)[:64]}]
        for index, kind in enumerate(("binaries", "maps", "routes", "policies", "drivers"), 1)
    }
    binding = "b" * 64
    return {
        "parentReleaseBindingSha256": binding,
        "parentReleaseBindingReceiptSha256": generate.canonical_hash({
            "parentReleaseBindingSha256": binding, "inventory": inventory}),
        "inventory": inventory,
        "restartReceipt": {
            "state": "DISARMED", "permissions": "UNKNOWN", "localizationRequired": True,
            "missionResume": False, "parentReleaseBindingSha256": binding,
            "inventoryDigest": generate.canonical_hash(inventory),
        },
    }


def complete_fixture(tmp_path):
    reports = release_tree(tmp_path)
    bindings = bindings_for(tmp_path, reports)
    for gate_id, relative in generate.REQUIRED_GATES.items():
        write_json(tmp_path / relative, gate_report(gate_id, bindings))
    return reports, rollback_binding(), bindings


def test_complete_ac0_ac6_fixture_derives_software_only_clean_authority(tmp_path):
    reports, rollback, bindings = complete_fixture(tmp_path)
    first = generate.generate_manifest(tmp_path, reports, rollback, source=SOURCE)
    second = generate.generate_manifest(tmp_path, reports, rollback, source=SOURCE)
    assert first == second
    assert first["gate_matrix"]["releaseBindings"] == bindings
    assert first["gate_matrix"]["passedGateIds"] == sorted(generate.REQUIRED_GATES)
    assert first["authority"] == {
        "software_release_candidate": True, "clean_release_authority": True,
        "hardware_motion_authorized": False, "passenger_operation_authorized": False,
        "physical_authority": False, "simulation_or_replay_is_physical_evidence": False,
    }
    assert first["qualification"] == {
        "target_nuc": "passed", "hardware": "blocked", "passenger": "blocked",
    }
    assert "target NUC qualification has not passed" not in first["known_blockers"]
    assert set(generate.DEFAULT_BLOCKERS) <= set(first["known_blockers"])
    assert generate.prepare_bindings(tmp_path, source=SOURCE) == bindings
def test_prepare_bindings_excludes_only_generated_gate_evidence(tmp_path):
    reports = release_tree(tmp_path)
    before = generate.prepare_bindings(tmp_path)
    for gate_id, relative in generate.REQUIRED_GATES.items():
        write_json(tmp_path / relative, gate_report(gate_id, before))
    assert generate.prepare_bindings(tmp_path) == before
    manifest = generate.generate_manifest(tmp_path, reports, rollback_binding())
    assert manifest["gate_matrix"]["releaseBindings"] == before

    (tmp_path / "src/example/config/settings.yaml").write_text("enabled: false\n", encoding="utf-8")
    assert generate.prepare_bindings(tmp_path) != before

    unexpected = tmp_path / "evidence/unexpected.json"
    unexpected.parent.mkdir(parents=True, exist_ok=True)
    unexpected.write_text("{}\n", encoding="utf-8")
    with pytest.raises(generate.ManifestError, match="unclassified regular file"):
        generate.prepare_bindings(tmp_path)


def test_prepare_bindings_cli_is_canonical_and_public(tmp_path):
    release_tree(tmp_path)
    command = [
        sys.executable, str(ROOT / "scripts/generate_release_manifest.py"),
        "--root", str(tmp_path), "--prepare-bindings",
    ]
    result = subprocess.run(command, check=True, stdout=subprocess.PIPE, text=True)
    bindings = json.loads(result.stdout)
    assert result.stdout == json.dumps(
        bindings, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n"
    assert bindings == generate.prepare_bindings(tmp_path)
def test_prepare_bindings_cli_supports_external_output(tmp_path):
    release_tree(tmp_path)
    output = tmp_path.parent / "bindings.json"
    command = [
        sys.executable, str(ROOT / "scripts/generate_release_manifest.py"),
        "--root", str(tmp_path), "--prepare-bindings", "--bindings-output", str(output),
    ]
    subprocess.run(command, check=True)
    assert json.loads(output.read_text(encoding="utf-8")) == generate.prepare_bindings(tmp_path)


def test_excluded_artifact_symlink_is_ignored_during_prepare_and_verify(tmp_path):
    output, manifest = manifest_fixture(tmp_path)
    link = tmp_path / "artifacts/qa/roslog/latest"
    link.parent.mkdir(parents=True)
    link.symlink_to(tmp_path.parent / "outside-release-root")
    assert generate.prepare_bindings(tmp_path, source=SOURCE)
    assert verify.verify_manifest(output, tmp_path) == manifest


def test_one_report_cannot_clean_pass(tmp_path):
    reports, rollback, _ = complete_fixture(tmp_path)
    with pytest.raises(generate.ManifestError, match="AC gate matrix is incomplete"):
        generate.generate_manifest(tmp_path, reports[:1], rollback, source=SOURCE)


@pytest.mark.parametrize("mutation, message", [
    (lambda report: report.update(sourceRevision="c" * 40), "stale or mixed"),
    (lambda report: report.update(configurationDigest="0" * 64), "stale or mixed"),
    (lambda report: report.update(bundleDigest="0" * 64), "stale or mixed"),
    (lambda report: report.update(releaseInputDigest="0" * 64), "stale or mixed"),
    (lambda report: report.update(claimTag="SIMULATION_ONLY"), "software-only"),
    (lambda report: report["result"].update(cases=0), "trivial"),
    (lambda report: report.update(gateId="WP7-PHYSICAL-001"), "ID/path"),
    (lambda report: report.update(hardwareMotionAuthorized=True), "authority must remain false"),
    (lambda report: report.update(extra="not allowed"), "non-strict schema"),
])
def test_gate_report_fail_closed_boundaries(tmp_path, mutation, message):
    reports, rollback, _ = complete_fixture(tmp_path)
    target = reports[0]
    document = json.loads(target.read_text(encoding="utf-8"))
    mutation(document)
    write_json(target, document)
    with pytest.raises(generate.ManifestError, match=message):
        generate.generate_manifest(tmp_path, reports, rollback, source=SOURCE)


def test_rejects_symlink_in_report_directory_ancestor(tmp_path):
    reports, rollback, _ = complete_fixture(tmp_path)
    evidence = tmp_path / "evidence"
    outside = tmp_path / "outside"
    evidence.rename(outside)
    evidence.symlink_to(outside, target_is_directory=True)
    with pytest.raises(generate.ManifestError, match="symlink is forbidden"):
        generate.generate_manifest(tmp_path, reports, rollback, source=SOURCE)


@pytest.mark.parametrize("mutation, message", [
    (lambda value: value["inventory"].pop("drivers"), "inventory is incomplete"),
    (lambda value: value.update(parentReleaseBindingReceiptSha256="0" * 64), "receipt does not bind"),
    (lambda value: value["restartReceipt"].update(state="ARMED"), "disarmed matching restart"),
    (lambda value: value["restartReceipt"].update(permissions="CLEAR"), "disarmed matching restart"),
    (lambda value: value["restartReceipt"].update(missionResume=True), "disarmed matching restart"),
])
def test_rollback_requires_signed_complete_disarmed_binding(tmp_path, mutation, message):
    reports, rollback, _ = complete_fixture(tmp_path)
    mutation(rollback)
    with pytest.raises(generate.ManifestError, match=message):
        generate.generate_manifest(tmp_path, reports, rollback, source=SOURCE)


def test_atomic_write_is_canonical(tmp_path):
    reports, rollback, _ = complete_fixture(tmp_path)
    manifest = generate.generate_manifest(tmp_path, reports, rollback, source=SOURCE)
    output = tmp_path / "release-manifest.json"
    generate.atomic_write(output, manifest)
    assert output.read_text(encoding="utf-8") == json.dumps(
        manifest, sort_keys=True, indent=2, ensure_ascii=False) + "\n"
def manifest_fixture(tmp_path):
    reports, rollback, _ = complete_fixture(tmp_path)
    manifest = generate.generate_manifest(tmp_path, reports, rollback, source=SOURCE)
    output = tmp_path / "release-manifest.json"
    generate.atomic_write(output, manifest)
    return output, manifest


def rebind(manifest):
    manifest.pop("release_binding_sha256")
    manifest["release_binding_sha256"] = generate.canonical_hash(manifest)
    return manifest


def test_generator_verifier_round_trip_and_representative_inventory(tmp_path):
    output, manifest = manifest_fixture(tmp_path)
    assert verify.verify_manifest(output, tmp_path) == manifest
    inventory = {category: {entry["path"] for entry in section["files"]}
                 for category, section in manifest["hashes"].items()}
    assert "src/example/CMakeLists.txt" in inventory["package_metadata"]
    assert "src/wheelchair_interfaces/msg/SafetyState.msg" in inventory["interfaces"]
    assert "src/example/config/settings.yaml" in inventory["configuration"]
    assert "data/site/map.pgm" in inventory["maps"]
    assert "data/site/site.waypoints.yaml" in inventory["routes"]
    assert "README.md" in inventory["operator_docs"]
    assert set(generate.REQUIRED_GATES.values()) == inventory["qualification_evidence"]


@pytest.mark.parametrize("entrypoint", generate.REQUIRED_RUNTIME_ENTRYPOINTS)
def test_verifier_rejects_missing_tampered_or_nonexecutable_runtime_entrypoint(tmp_path, entrypoint):
    output, _ = manifest_fixture(tmp_path)
    (tmp_path / entrypoint).unlink()
    with pytest.raises(verify.ManifestError, match="missing|required runtime"):
        verify.verify_manifest(output, tmp_path)

    output, _ = manifest_fixture(tmp_path)
    candidate = tmp_path / entrypoint
    candidate.write_text("#!/usr/bin/env python3\nprint('tampered')\n", encoding="utf-8")
    with pytest.raises(verify.ManifestError, match="hash mismatch"):
        verify.verify_manifest(output, tmp_path)

    output, _ = manifest_fixture(tmp_path)
    (tmp_path / entrypoint).chmod(0o644)
    with pytest.raises(verify.ManifestError, match="executable mode|required runtime"):
        verify.verify_manifest(output, tmp_path)


@pytest.mark.parametrize("relative", ["src/example/config/unbound.yaml", "unclassified.future"])
def test_verifier_rejects_unbound_or_unclassified_files(tmp_path, relative):
    output, _ = manifest_fixture(tmp_path)
    candidate = tmp_path / relative
    candidate.parent.mkdir(parents=True, exist_ok=True)
    candidate.write_text("unsafe: true\n", encoding="utf-8")
    with pytest.raises(verify.ManifestError, match="hash scope differs|unclassified regular file"):
        verify.verify_manifest(output, tmp_path)


def test_verifier_rejects_missing_semantic_and_authority_rebound_tamper(tmp_path):
    output, manifest = manifest_fixture(tmp_path)
    (tmp_path / next(iter(generate.REQUIRED_GATES.values()))).unlink()
    with pytest.raises(verify.ManifestError, match="missing test report|missing gate report"):
        verify.verify_manifest(output, tmp_path)

    output, manifest = manifest_fixture(tmp_path)
    report_path = tmp_path / next(iter(generate.REQUIRED_GATES.values()))
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["result"]["passed"] = False
    write_json(report_path, report)
    for entry in manifest["test_reports"] + manifest["hashes"]["qualification_evidence"]["files"]:
        if entry["path"] == report_path.relative_to(tmp_path).as_posix():
            entry["sha256"] = generate.file_hash(report_path)
    entries = manifest["hashes"]["qualification_evidence"]["files"]
    manifest["hashes"]["qualification_evidence"]["digest"] = generate.canonical_hash(entries)
    generate.atomic_write(output, rebind(manifest))
    with pytest.raises(verify.ManifestError, match="invalid or trivial result"):
        verify.verify_manifest(output, tmp_path)

    output, manifest = manifest_fixture(tmp_path)
    manifest["qualification"]["target_nuc"] = "blocked"
    manifest["known_blockers"].append("target NUC qualification has not passed")
    manifest["known_blockers"].sort()
    generate.atomic_write(output, rebind(manifest))
    with pytest.raises(verify.ManifestError, match="qualification status contradicts"):
        verify.verify_manifest(output, tmp_path)
    output, manifest = manifest_fixture(tmp_path)
    manifest["authority"]["hardware_motion_authorized"] = True
    generate.atomic_write(output, rebind(manifest))
    with pytest.raises(verify.ManifestError, match="authority escalation"):
        verify.verify_manifest(output, tmp_path)


def test_verifier_rejects_matrix_and_rollback_rebinding_tamper(tmp_path):
    output, manifest = manifest_fixture(tmp_path)
    manifest["gate_matrix"]["releaseBindings"]["bundleDigest"] = "0" * 64
    generate.atomic_write(output, rebind(manifest))
    with pytest.raises(verify.ManifestError, match="matrix is incomplete or stale"):
        verify.verify_manifest(output, tmp_path)

    output, manifest = manifest_fixture(tmp_path)
    manifest["rollback"]["restartReceipt"]["state"] = "ARMED"
    generate.atomic_write(output, rebind(manifest))
    with pytest.raises(verify.ManifestError, match="disarmed matching restart"):
        verify.verify_manifest(output, tmp_path)


def test_verifier_rejects_symlink_file_and_directory_ancestor(tmp_path):
    output, _ = manifest_fixture(tmp_path)
    report = tmp_path / next(iter(generate.REQUIRED_GATES.values()))
    replacement = tmp_path / "replacement.json"
    replacement.write_text(report.read_text(encoding="utf-8"), encoding="utf-8")
    report.unlink()
    report.symlink_to(replacement)
    with pytest.raises(verify.ManifestError, match="symlink is forbidden"):
        verify.verify_manifest(output, tmp_path)
    report.unlink()
    replacement.unlink()

    output, _ = manifest_fixture(tmp_path)
    evidence = tmp_path / "evidence"
    outside = tmp_path / "outside"
    evidence.rename(outside)
    evidence.symlink_to(outside, target_is_directory=True)
    with pytest.raises(verify.ManifestError, match="symlink is forbidden"):
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
