#!/usr/bin/env python3
"""Filesystem-only tests for atomic Noetic RC installation and rollback."""

import hashlib
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(name[:-3], str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


INSTALL = load_script("install_noetic_rc.py")
ROLLBACK = load_script("rollback_noetic_rc.py")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def make_release(base, name, parent="bootstrap", authority=None):
    root = base / name
    files = {
        "src/pkg/package.xml": b"<package/>\n",
        "src/pkg/config/policy.yaml": ("release: " + name + "\n").encode(),
        "data/map.pgm": b"P2\n1 1\n255\n0\n",
        "data/map.yaml": b"image: map.pgm\n",
        "data/route-waypoints.yaml": b"waypoints: []\n",
        "contracts/contract.json": b"{}\n",
        "tools/noetic/Dockerfile": b"FROM scratch\n",
        "reports/test.json": b"{\"passed\":true}\n",
    }
    for relative, content in files.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    categories = {
        "package_metadata": ["src/pkg/package.xml"],
        "configuration": ["src/pkg/config/policy.yaml"],
        "contracts": ["contracts/contract.json"],
        "maps": ["data/map.pgm", "data/map.yaml"],
        "routes": ["data/route-waypoints.yaml"],
        "ci_tools": ["tools/noetic/Dockerfile"],
        "qualification_evidence": ["reports/test.json"],
    }
    hashes = {}
    for category, paths in categories.items():
        entries = [{"path": item, "sha256": digest(root / item)} for item in paths]
        hashes[category] = {"files": entries, "digest": canonical(entries)}
    manifest = {
        "schema": "wheelchair-noetic-release-manifest/v1",
        "source": {"kind": "test", "revision": name, "worktree_clean": True},
        "hashes": hashes,
        "authority": authority or {
            "software_release_candidate": True,
            "clean_release_authority": True,
            "hardware_motion_authorized": False,
            "passenger_operation_authorized": False,
            "physical_authority": False,
            "simulation_or_replay_is_physical_evidence": False,
            "hardware_enabled": False,
        },
        "qualification": {"target_nuc": "blocked", "hardware": "blocked", "passenger": "blocked"},
        "test_reports": [{"path": "reports/test.json", "sha256": digest(root / "reports/test.json")}],
        "known_blockers": ["hardware qualification not performed"],
        "rollback": {"parent": parent, "parent_state": "unarmed"},
    }
    manifest["release_binding_sha256"] = canonical(manifest)
    path = root / "manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return root, path, manifest


def verify_manifest(manifest_path, root):
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    binding = manifest.pop("release_binding_sha256", None)
    if binding != canonical(manifest):
        raise INSTALL.InstallError("invalid manifest binding")
    manifest["release_binding_sha256"] = binding
    entries = []
    for category in manifest["hashes"].values():
        entries.extend(category["files"])
    entries.extend(manifest["test_reports"])
    for entry in entries:
        relative = Path(entry["path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise INSTALL.InstallError("unsafe path")
        path = Path(root) / relative
        if not path.is_file() or digest(path) != entry["sha256"]:
            raise INSTALL.InstallError("hash mismatch")
    return manifest


class DeploymentRollbackTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.base = Path(self.temporary.name)
        self.prefix = self.base / "sandbox"
        self.non_root = mock.patch.object(os, "geteuid", return_value=1000)
        self.non_root.start()

    def tearDown(self):
        self.non_root.stop()
        self.temporary.cleanup()

    def install(self, release):
        root, manifest_path, manifest = release
        result = INSTALL.install_release(root, manifest_path, self.prefix, True, verify_manifest)
        return result, manifest

    def test_dry_run_is_default_and_does_not_create_prefix(self):
        root, manifest_path, _ = make_release(self.base, "dry")
        result = INSTALL.install_release(root, manifest_path, self.prefix, verifier=verify_manifest)
        self.assertFalse(result["applied"])
        self.assertFalse(self.prefix.exists())
        self.assertEqual("DISARMED", result["state"])

    def test_install_is_atomic_and_idempotent(self):
        release = make_release(self.base, "one")
        first, manifest = self.install(release)
        self.assertEqual("releases/" + manifest["release_binding_sha256"], os.readlink(self.prefix / "current"))
        second, _ = self.install(release)
        self.assertEqual(first["release"], second["release"])
        self.assertEqual("DISARMED", second["state"])
        self.assertTrue((self.prefix / "receipts" / ("install-" + first["release"] + ".json")).is_file())


    def test_installed_contents_equal_manifest_inventory(self):
        result, manifest = self.install(make_release(self.base, "inventory"))
        final = self.prefix / "releases" / result["release"]
        expected = {
            entry["path"]
            for section in manifest["hashes"].values()
            for entry in section["files"]
        }
        observed = {
            path.relative_to(final).as_posix()
            for path in final.rglob("*")
            if path.is_file()
        }
        self.assertEqual(expected | {"release-manifest.json"}, observed)
        self.assertEqual(sorted(expected), result["files"])

    def test_tampered_source_is_rejected_without_changing_current(self):
        good = make_release(self.base, "good")
        installed, _ = self.install(good)
        bad = make_release(self.base, "bad")
        (bad[0] / "src/pkg/config/policy.yaml").write_text("tampered\n", encoding="utf-8")
        with self.assertRaises(INSTALL.InstallError):
            self.install(bad)
        self.assertTrue(os.readlink(self.prefix / "current").endswith(installed["release"]))

    def test_interrupted_staging_leaves_prior_current(self):
        prior, _ = self.install(make_release(self.base, "prior"))
        root, manifest_path, _ = make_release(self.base, "interrupted")
        def interrupt(point, unused_path):
            if point == "staged":
                raise RuntimeError("simulated interruption")
        with self.assertRaises(RuntimeError):
            INSTALL.install_release(root, manifest_path, self.prefix, True, verify_manifest, interrupt)
        self.assertTrue(os.readlink(self.prefix / "current").endswith(prior["release"]))
        self.assertTrue(any(path.name.startswith(".") and ".staging-" in path.name
                            for path in (self.prefix / "releases").iterdir()))

    def test_rollback_verifies_parent_and_returns_disarmed(self):
        parent_result, parent = self.install(make_release(self.base, "parent"))
        child_result, _ = self.install(make_release(self.base, "child", parent_result["release"]))
        result = ROLLBACK.rollback_release(self.prefix, parent_result["release"], "DISARMED",
                                           True, verify_manifest)
        self.assertEqual("DISARMED", result["state"])
        self.assertEqual(parent_result["release"], result["to_release"])
        self.assertTrue(os.readlink(self.prefix / "current").endswith(parent["release_binding_sha256"]))
        receipt = self.prefix / "receipts" / ("rollback-{}-to-{}.json".format(
            child_result["release"], parent_result["release"]))
        self.assertEqual("DISARMED", json.loads(receipt.read_text())["armed_state"])
        repeated = ROLLBACK.rollback_release(self.prefix, parent_result["release"], "DISARMED",
                                              True, verify_manifest)
        self.assertTrue(repeated["idempotent"])

    def test_armed_manifest_and_armed_rollback_are_rejected(self):
        authority = {
            "software_release_candidate": True, "clean_release_authority": True,
            "hardware_motion_authorized": True, "passenger_operation_authorized": False,
            "physical_authority": False, "simulation_or_replay_is_physical_evidence": False,
            "hardware_enabled": True,
        }
        with self.assertRaises(INSTALL.InstallError):
            self.install(make_release(self.base, "armed", authority=authority))
        parent, _ = self.install(make_release(self.base, "safe-parent"))
        self.install(make_release(self.base, "safe-child", parent["release"]))
        with self.assertRaises(ROLLBACK.RollbackError):
            ROLLBACK.rollback_release(self.prefix, parent["release"], "ARMED", True, verify_manifest)

    def test_manifest_path_traversal_is_rejected(self):
        root, manifest_path, manifest = make_release(self.base, "traversal")
        manifest["hashes"]["configuration"]["files"][0]["path"] = "../outside.yaml"
        manifest["release_binding_sha256"] = canonical({k: v for k, v in manifest.items()
                                                         if k != "release_binding_sha256"})
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
        with self.assertRaises(INSTALL.InstallError):
            INSTALL.install_release(root, manifest_path, self.prefix, True, verify_manifest)
        self.assertFalse(self.prefix.exists())


if __name__ == "__main__":
    unittest.main()
