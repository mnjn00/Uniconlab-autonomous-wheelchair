"""Adversarial, ROS-free tests for the WP0 contract validator."""

import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


REPOSITORY = Path(__file__).resolve().parents[1]
VALIDATOR = REPOSITORY / "scripts" / "validate_wp0_contracts.py"


class WP0ContractValidatorTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        shutil.copytree(REPOSITORY / "contracts", self.root / "contracts")
        shutil.copytree(REPOSITORY / "data", self.root / "data")
        shutil.copy2(REPOSITORY / "README.md", self.root / "README.md")

    def tearDown(self):
        self.temporary.cleanup()

    @property
    def contract_dir(self):
        return self.root / "contracts" / "wp0"

    def run_validator(self):
        return subprocess.run(
            [sys.executable, str(VALIDATOR), "--root", str(self.root)],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=30,
        )

    def replace(self, artifact, old, new):
        path = self.contract_dir / artifact
        content = path.read_text(encoding="utf-8")
        self.assertIn(old, content)
        path.write_text(content.replace(old, new, 1), encoding="utf-8")
        self.refresh_manifest_hash(artifact)

    def refresh_manifest_hash(self, artifact):
        digest = hashlib.sha256((self.contract_dir / artifact).read_bytes()).hexdigest()
        manifest_path = self.contract_dir / "manifest.yaml"
        lines = manifest_path.read_text(encoding="utf-8").splitlines()
        marker = f"  - path: {artifact}"
        index = lines.index(marker)
        self.assertTrue(lines[index + 1].startswith("    sha256: "))
        lines[index + 1] = "    sha256: " + digest
        manifest_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def assert_rejected(self, result, diagnostic):
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn(diagnostic, result.stderr)

    def test_clean_contracts_pass(self):
        result = self.run_validator()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "WP0 contracts valid")

    def test_tampered_release_authority_fails_closed(self):
        self.replace(
            "A16-release-authority.yaml",
            "  hardware_motion_authorized: false",
            "  hardware_motion_authorized: true",
        )
        self.assert_rejected(self.run_validator(), "E_AUTHORITY: A16-release-authority.yaml")

    def test_tampered_reason_bit_is_rejected(self):
        self.replace(
            "A04-safety-reason-registry.yaml",
            "{bit: 12, name: SENSOR_STALE, value: 4096",
            "{bit: 12, name: SENSOR_STALE, value: 8192",
        )
        result = self.run_validator()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertEqual(
            result.stderr.strip(),
            "E_REASON_VALUE: A04-safety-reason-registry.yaml: "
            "reason names, bits, and values must exactly match SafetyReason ABI v1",
        )

    def test_tampered_schema_hash_binding_is_rejected(self):
        policy = yaml.safe_load(
            (self.contract_dir / "collision-simulation-policy.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.replace(
            "collision-simulation-policy.yaml",
            policy["hashes"]["schema_sha256"]["value"],
            "1" * 64,
        )
        self.assert_rejected(self.run_validator(), "E_SCHEMA_HASH: collision-simulation-policy.yaml")

    def test_unverified_real_motor_topic_is_rejected(self):
        self.replace(
            "driver-unverified.yaml",
            "  driver_topic: ''",
            "  driver_topic: /wheelchair/motor_cmd",
        )
        self.assert_rejected(self.run_validator(), "E_MOTOR_TOPIC: driver-unverified.yaml")

    def test_missing_required_artifact_is_rejected(self):
        (self.contract_dir / "A10-conversion-abi-v1.md").unlink()
        self.assert_rejected(self.run_validator(), "E_MISSING_ARTIFACT: A10-conversion-abi-v1.md")

    def test_full_bag_normalization_evidence_is_hash_bound(self):
        inventory = yaml.safe_load(
            (REPOSITORY / "contracts" / "wp0" / "A15-evidence-inventory.yaml").read_text(
                encoding="utf-8"
            )
        )
        normalization = inventory["source_dataset"]["full_bag_normalization"]
        evidence_path = REPOSITORY / normalization["evidence_path"]
        self.assertEqual(
            hashlib.sha256(evidence_path.read_bytes()).hexdigest(),
            normalization["evidence_sha256"],
        )

    def test_full_bag_claims_remain_narrow_and_fail_closed(self):
        evidence = json.loads(
            (
                REPOSITORY
                / "artifacts"
                / "software_rc"
                / "full-bag-normalization.json"
            ).read_text(encoding="utf-8")
        )
        self.assertEqual(evidence["verifier"]["status"], "ok")
        self.assertEqual(evidence["repeatability"]["independent_conversion_count"], 3)
        self.assertTrue(evidence["repeatability"]["byte_identical_outputs"])
        self.assertEqual(
            evidence["outputs"]["counts"],
            {
                "clouds": 6882,
                "imus": 137602,
                "total_records": 144484,
                "points": 137594880,
                "clouds_with_adjacent_offset_decrease": 6882,
                "adjacent_offset_decreases": 910296,
                "points_sorted": 0,
                "points_dropped": 0,
                "points_repaired": 0,
            },
        )
        self.assertFalse(evidence["inputs"]["alignment"]["verified"])
        self.assertFalse(
            evidence["inputs"]["converter"][
                "typestore_is_recording_distribution_evidence"
            ]
        )
        self.assertIn("sensor fusion", evidence["claim_limits"]["not_qualified"])
        self.assertIn(
            "source recording ROS distribution", evidence["claim_limits"]["unknown"]
        )
        self.assertEqual(
            evidence["authority"],
            {
                "hardware_motion_authorized": False,
                "passenger_operation_authorized": False,
            },
        )


if __name__ == "__main__":
    unittest.main()
