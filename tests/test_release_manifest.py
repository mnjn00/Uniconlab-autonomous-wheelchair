import importlib.util
import json
from pathlib import Path
import pytest
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / (name + ".py")); module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module
generate = load("generate_release_manifest"); verify = load("verify_release_manifest")
def dump(path, value): path.parent.mkdir(parents=True, exist_ok=True); path.write_text(json.dumps(value, sort_keys=True) + "\n")

def tree(root):
    files = {".dockerignore":"build\n", "src/example/package.xml":"<package/>\n", "src/example/CMakeLists.txt":"cmake_minimum_required(VERSION 3.0)\n", "src/example/setup.py":"x=1\n", "src/wheelchair_interfaces/msg/State.msg":"bool x\n", "src/example/config/settings.yaml":"safe: true\n", "src/example/launch/a.launch":"<launch/>\n", "src/wheelchair_description/urdf/a.urdf":"<robot/>\n", "contracts/a.json":"{}\n", "data/a.pgm":"P2\n1 1\n255\n0\n", "data/a.waypoints.yaml":"waypoints: []\n", "README.md":"# guide\n", ".github/workflows/a.yml":"name: ci\n", "tests/test_a.py":"pass\n"}
    for entrypoint in generate.REQUIRED_RUNTIME_ENTRYPOINTS: files[entrypoint] = "#!/usr/bin/env python3\n"
    for relative, content in files.items():
        path=root/relative; path.parent.mkdir(parents=True, exist_ok=True); path.write_text(content)
    for entrypoint in generate.REQUIRED_RUNTIME_ENTRYPOINTS: (root/entrypoint).chmod(0o755)
    for gate, report in generate.REQUIRED_GATES.items():
        artifact=root/"evidence/artifacts"/(gate+".txt"); artifact.parent.mkdir(parents=True, exist_ok=True); artifact.write_text("measured " + gate + "\n")
        dump(root/report, {})

def report(root, gate, bindings):
    artifact="evidence/artifacts/" + gate + ".txt"; requirement=generate.GATE_REQUIREMENTS[gate]
    return {"artifactType":"wheelchair-ac-gate-report", "schemaVersion":2, "gateId":gate, "status":"PASS", "claimTag":"SOFTWARE_ONLY", "hardwareMotionAuthorized":False, "passengerOperationAuthorized":False, **bindings, "result":{"passed":True, "executedCommands":["pytest -q " + gate], "environment":{"os":"ubuntu", "architecture":"x86_64"}, "tool":{"name":"pytest", "version":"8"}, "durationSeconds":1.0, "metrics":{requirement["metric"]:1}, "invariants":{requirement["invariant"]:True}, "artifacts":[{"path":artifact, "sha256":generate.file_hash(root/artifact)}]}}

def rollback():
    return {"parentReleaseBindingSha256":"a"*64, "parentManifestSha256":"b"*64, "parentManifestPath":"release-manifest.json", "parentInventoryDigest":"c"*64, "restartReceipt":{"path":"evidence/release/parent-restart.json", "sha256":"d"*64, "parentReleaseBindingSha256":"a"*64, "parentInventoryDigest":"c"*64}}

def fixture(root):
    tree(root); bindings=generate.prepare_bindings(root)
    for gate, path in generate.REQUIRED_GATES.items(): dump(root/path, report(root, gate, bindings))
    return [root/path for path in generate.REQUIRED_GATES.values()], bindings

def test_draft_requires_complete_typed_ac_matrix_and_stays_non_authoritative(tmp_path):
    reports, bindings=fixture(tmp_path); manifest=generate.generate_manifest(tmp_path, reports, rollback())
    assert manifest["authority"]["clean_release_authority"] is False
    assert manifest["source"]["kind"] == "worktree"
    assert manifest["gate_matrix"]["releaseBindings"] == bindings
    out=tmp_path/"release-manifest.json"; generate.atomic_write(out, manifest)
    assert verify.verify_manifest(out, tmp_path) == manifest

@pytest.mark.parametrize("gate", list(generate.REQUIRED_GATES))
def test_each_gate_rejects_missing_metric_command_environment_and_artifact(tmp_path, gate):
    reports, bindings=fixture(tmp_path); path=tmp_path/generate.REQUIRED_GATES[gate]; value=json.loads(path.read_text())
    value["result"]["metrics"] = {}; dump(path, value)
    with pytest.raises(generate.ManifestError, match="metrics"): generate.generate_manifest(tmp_path, reports, rollback())
    dump(path, report(tmp_path, gate, bindings)); value=json.loads(path.read_text()); value["result"]["executedCommands"] = []; dump(path, value)
    with pytest.raises(generate.ManifestError, match="commands"): generate.generate_manifest(tmp_path, reports, rollback())
    dump(path, report(tmp_path, gate, bindings)); value=json.loads(path.read_text()); value["result"]["artifacts"][0]["sha256"]="0"*64; dump(path, value)
    with pytest.raises(generate.ManifestError, match="artifact hash"): generate.generate_manifest(tmp_path, reports, rollback())

def test_missing_glim_unknown_blocker_and_cross_bundle_evidence_rejected(tmp_path):
    reports, bindings=fixture(tmp_path)
    missing=[p for p in reports if "glim-offline-input" not in str(p)]
    with pytest.raises(generate.ManifestError, match="matrix"): generate.generate_manifest(tmp_path, missing, rollback())
    with pytest.raises(generate.ManifestError, match="blockers"): generate.generate_manifest(tmp_path, reports, rollback(), blockers=["operator said okay"])
    path=tmp_path/generate.REQUIRED_GATES["WP3-GLIM-COMPARISON-001"]; value=json.loads(path.read_text()); value["bundleDigest"]="0"*64; dump(path, value)
    with pytest.raises(generate.ManifestError, match="mixed"): generate.generate_manifest(tmp_path, reports, rollback())

def test_authoritative_generation_requires_clean_git_and_key(tmp_path, monkeypatch):
    reports, _=fixture(tmp_path)
    monkeypatch.setattr(generate, "source_identity", lambda root: {"kind":"git_commit", "revision":"a"*40, "worktree_clean":True})
    with pytest.raises(generate.ManifestError, match="signing key"): generate.generate_manifest(tmp_path, reports, rollback())
