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
        image_digest = "a" * 64
        image = "registry.example/glim@sha256:" + image_digest
        input_dir = root / "input"
        input_dir.mkdir()
        input_manifest = input_dir / "glim_rosbag2_manifest.json"
        input_manifest.write_text("{}", encoding="utf-8")
        database = input_dir / "normalized.db3"
        database.write_bytes(b"database")
        config_dir = root / "config"
        config_dir.mkdir()
        config_manifest = config_dir / "manifest.json"
        config_manifest.write_text("{}", encoding="utf-8")
        for name in ("config.json", "config_preprocess.json", "config_odometry_cpu.json", "config_global_mapping_cpu.json"):
            (config_dir / name).write_text("{}", encoding="utf-8")
        command = [
            "docker", "run", "--network=none", "--read-only", "--security-opt=no-new-privileges",
            "--cap-drop=ALL", "--env=OMP_NUM_THREADS=1", "--env=OPENBLAS_NUM_THREADS=1",
            "--env=MKL_NUM_THREADS=1", "--label=wheelchair.offline-only=true",
            "--mount=type=bind,src=%s,dst=/input/rosbag2,readonly" % input_dir,
            "--mount=type=bind,src=%s,dst=/opt/glim-config/config.json,readonly" % (config_dir / "config.json"),
            "--mount=type=bind,src=%s,dst=/opt/glim-config/config_preprocess.json,readonly" % (config_dir / "config_preprocess.json"),
            "--mount=type=bind,src=%s,dst=/opt/glim-config/config_odometry_cpu.json,readonly" % (config_dir / "config_odometry_cpu.json"),
            "--mount=type=bind,src=%s,dst=/opt/glim-config/config_global_mapping_cpu.json,readonly" % (config_dir / "config_global_mapping_cpu.json"),
            image,
        ]
        runs = []
        for run_id, (poses, pixels, status) in enumerate(zip(trajectories, maps, statuses), 1):
            directory = root / ("run-%02d" % run_id)
            directory.mkdir()
            trajectory = directory / "trajectory.csv"
            trajectory.write_text("timestamp,x,y,yaw\n" + "".join("%s,%s,%s,0\n" % pose for pose in poses), encoding="utf-8")
            pgm = directory / "occupancy.pgm"
            pgm.write_text("P2\n2 2\n255\n%s\n" % pixels, encoding="ascii")
            metadata = directory / "occupancy.yaml"
            metadata.write_text("resolution: 1.0\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\n", encoding="utf-8")
            (directory / "actual_output_evidence.json").write_text("{}", encoding="utf-8")
            (directory / "stdout.log").write_text("", encoding="utf-8")
            (directory / "stderr.log").write_text("", encoding="utf-8")
            artifacts = {
                path.name: {"sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "size_bytes": path.stat().st_size}
                for path in directory.iterdir()
            }
            receipt = {
                "run_id": run_id, "directory": directory.name, "status": status,
                "failure": None, "pseudo_point_time_diagnostics": [], "returncode": 0,
                "command": command, "image": image, "source_revision": "b" * 40,
                "glim_ros2_revision": "c" * 40, "seed": 20260707, "threads": 1,
                "elapsed_s": 1.0, "artifacts": artifacts,
            }
            (directory / "run_manifest.json").write_text(json.dumps(receipt), encoding="utf-8")
            runs.append(receipt)
        root_receipt = {
            "schema_version": 2, "artifact_id": "wheelchair.glim-reproduction/v2", "status": "success",
            "execution_scope": "OFFLINE_WORKSTATION_ONLY", "nuc_runtime_dependency": False,
            "claim_label": "REPLAY_CONSISTENCY_NOT_TRUTH", "qualification": "candidate",
            "image": image, "source_revision": "b" * 40, "glim_ros2_revision": "c" * 40,
            "seed": 20260707, "threads": 1,
            "input_manifest_sha256": hashlib.sha256(input_manifest.read_bytes()).hexdigest(),
            "ros2_database_sha256": hashlib.sha256(database.read_bytes()).hexdigest(),
            "config_sha256": hashlib.sha256(config_manifest.read_bytes()).hexdigest(),
            "config_entrypoint_sha256": hashlib.sha256((config_dir / "config.json").read_bytes()).hexdigest(),
            "runs": runs,
        }
        (root / "repro_manifest.json").write_text(json.dumps(root_receipt), encoding="utf-8")

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


    def test_strict_receipt_rejects_immutable_and_path_attacks(self):
        cases = ("skeletal", "input", "config", "image", "seed", "threads", "escape", "tolerance")
        for kind in cases:
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                self.write_artifact_tree(root)
                manifest_path = root / "repro_manifest.json"
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                if kind == "skeletal":
                    manifest["runs"][0]["artifacts"].pop("stdout.log")
                elif kind == "escape":
                    manifest["runs"][0]["directory"] = "../run-01"
                elif kind == "input":
                    manifest["input_manifest_sha256"] = "d" * 64
                elif kind == "config":
                    manifest["runs"][2]["command"][11] = "--mount=type=bind,src=/other/config.json,dst=/opt/glim-config/config.json,readonly"
                elif kind == "image":
                    manifest["runs"][2]["image"] = "registry.example/glim@sha256:" + "d" * 64
                elif kind == "seed":
                    manifest["runs"][2]["seed"] = 7
                elif kind == "threads":
                    manifest["runs"][2]["threads"] = 2
                if kind != "tolerance":
                    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
                    if kind not in ("input",):
                        (root / "run-01/run_manifest.json").write_text(json.dumps(manifest["runs"][0]), encoding="utf-8")
                        if kind in ("config", "image", "seed", "threads"):
                            (root / "run-03/run_manifest.json").write_text(json.dumps(manifest["runs"][2]), encoding="utf-8")
                    exit_code, report = self.compare(root)
                else:
                    output = root / "report.json"
                    exit_code = comparator.main(["--repro-dir", str(root), "--output", str(output), "--time-tolerance-s", "0.01"])
                    report = json.loads(output.read_text(encoding="utf-8"))
                self.assertNotEqual(exit_code, 0)
                self.assertEqual(report["status"], "fail")

    def test_symlinked_run_or_artifact_fails(self):
        for kind in ("run", "artifact"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                self.write_artifact_tree(root)
                if kind == "run":
                    (root / "run-01").rename(root / "real-run")
                    (root / "run-01").symlink_to(root / "real-run", target_is_directory=True)
                else:
                    target = root / "run-01/trajectory.csv"
                    target.rename(root / "run-01/real-trajectory.csv")
                    target.symlink_to(root / "run-01/real-trajectory.csv")
                exit_code, report = self.compare(root)
                self.assertNotEqual(exit_code, 0)
                self.assertTrue(report["errors"])
if __name__ == "__main__":
    unittest.main()
