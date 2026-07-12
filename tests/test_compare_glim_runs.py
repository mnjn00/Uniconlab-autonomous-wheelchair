"""Pure regression coverage for AC3 GLIM comparison report semantics."""
import hashlib
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COMPARATOR_PATH = ROOT / "scripts/compare_glim_runs.py"


def load_comparator():
    spec = importlib.util.spec_from_file_location("compare_glim_runs", COMPARATOR_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


comparator = load_comparator()


class CompareGlimRunsTests(unittest.TestCase):
    def write_artifact_tree(self, root, trajectories=None, maps=None, statuses=None):
        trajectories = trajectories or [
            [(0, 0, 0), (1, 1, 0), (2, 1, 1)],
            [(0, 0, 0), (1, 1, 0), (2, 1, 1)],
            [(0, 0, 0), (1, 1, 0), (2, 1, 1)],
        ]
        maps = maps or ["0 0 0 0", "0 0 0 0", "0 0 0 0"]
        statuses = statuses or ["success", "success", "success"]
        runs = []
        for run_id, (poses, pixels, status) in enumerate(zip(trajectories, maps, statuses), 1):
            directory = root / ("run-%02d" % run_id)
            directory.mkdir()
            trajectory = directory / "trajectory.csv"
            trajectory.write_text(
                "timestamp,x,y,yaw\n" + "".join(
                    "%s,%s,%s,0\n" % pose for pose in poses
                ),
                encoding="utf-8",
            )
            pgm = directory / "occupancy.pgm"
            pgm.write_text("P2\n2 2\n255\n%s\n" % pixels, encoding="ascii")
            metadata = directory / "occupancy.yaml"
            metadata.write_text(
                "resolution: 1.0\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\n",
                encoding="utf-8",
            )
            artifacts = {}
            for path in (trajectory, pgm, metadata):
                artifacts[path.name] = {"sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
            runs.append({
                "run_id": run_id,
                "directory": directory.name,
                "status": status,
                "artifacts": artifacts,
            })
        (root / "repro_manifest.json").write_text(json.dumps({"runs": runs}), encoding="utf-8")

    def compare(self, root):
        output = root / "report.json"
        exit_code = comparator.main(["--repro-dir", str(root), "--output", str(output)])
        return exit_code, json.loads(output.read_text(encoding="utf-8"))

    def test_repeatability_passes_when_loop_target_is_false(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write_artifact_tree(root)
            exit_code, report = self.compare(root)

        self.assertEqual(exit_code, 0)
        self.assertEqual(report["status"], "pass")
        self.assertFalse(report["loop_target_met_all_runs"])
        self.assertTrue(all(pair["ac3_repeatability_pass"] for pair in report["pairs"]))
        self.assertEqual(report["qualification"], "candidate")
        self.assertEqual(report["claim_label"], "REPLAY_CONSISTENCY_NOT_TRUTH")
        self.assertFalse(report["hardware_localization_accuracy_qualified"])

    def test_one_pair_rms_or_iou_failure_fails_repeatability(self):
        cases = {
            "rms": {
                "trajectories": [
                    [(0, 0, 0), (1, 1, 0), (2, 1, 1)],
                    [(0, 0, 0), (1, 1, 0), (2, 1, 1)],
                    [(0, 0, 0), (1, 1, 0), (2, 3, 1)],
                ],
            },
            "iou": {"maps": ["0 0 0 0", "0 0 0 0", "0 0 0 255"]},
        }
        for name, changes in cases.items():
            with self.subTest(gate=name), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                self.write_artifact_tree(root, **changes)
                exit_code, report = self.compare(root)

                self.assertNotEqual(exit_code, 0)
                self.assertEqual(report["status"], "fail")
                self.assertTrue(any(not pair["ac3_repeatability_pass"] for pair in report["pairs"]))

    def test_invalid_run_structure_or_integrity_fails(self):
        for kind in ("malformed", "missing", "hash_mismatch", "failed"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                statuses = ["success", "success", "failed"] if kind == "failed" else None
                self.write_artifact_tree(root, statuses=statuses)
                trajectory = root / "run-03/trajectory.csv"
                if kind == "malformed":
                    trajectory.write_text("timestamp,x,y\n0,0,0\n", encoding="utf-8")
                    manifest = json.loads((root / "repro_manifest.json").read_text(encoding="utf-8"))
                    manifest["runs"][2]["artifacts"]["trajectory.csv"]["sha256"] = hashlib.sha256(
                        trajectory.read_bytes()
                    ).hexdigest()
                    (root / "repro_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
                elif kind == "missing":
                    (root / "run-03/occupancy.pgm").unlink()
                elif kind == "hash_mismatch":
                    trajectory.write_text(trajectory.read_text(encoding="utf-8") + "3,1,1,0\n", encoding="utf-8")

                exit_code, report = self.compare(root)
                self.assertNotEqual(exit_code, 0)
                self.assertEqual(report["status"], "fail")
                self.assertTrue(report["errors"])

    def test_loop_diagnostic_preserves_numeric_values_and_limitation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write_artifact_tree(root)
            _, report = self.compare(root)

        diagnostic = report["runs"][0]["loop_residual"]
        self.assertAlmostEqual(diagnostic["position_residual_m"], math.sqrt(2.0))
        self.assertEqual(diagnostic["yaw_residual_deg"], 0.0)
        self.assertFalse(diagnostic["target_met"])
        self.assertIn("diagnostic only", diagnostic["interpretation"])
        self.assertTrue(any("diagnostic target" in limitation for limitation in report["limitations"]))


if __name__ == "__main__":
    unittest.main()
