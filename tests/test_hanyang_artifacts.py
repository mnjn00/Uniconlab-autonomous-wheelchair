"""Static checks that keep current Hanyang artifacts honestly candidate-only."""

import hashlib
import importlib.util
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "hanyang_aegimun_loop"
SPEC = importlib.util.spec_from_file_location("validate_hanyang", ROOT / "scripts" / "validate_hanyang_route.py")
VALIDATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATE)


class HanyangArtifactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.metadata = json.loads((DATA / "map.metadata.json").read_text(encoding="utf-8"))
        cls.report = VALIDATE.validate(DATA / "map.yaml", DATA / "hanyang_aegimun_loop.waypoints.yaml",
                                       DATA / "map.metadata.json")
        cls.route = VALIDATE.load_yaml(DATA / "hanyang_aegimun_loop.waypoints.yaml")

    def test_regenerated_pgm_dimensions_match_metadata(self):
        self.assertEqual((self.report["map"]["width"], self.report["map"]["height"]), (3220, 2361))
        self.assertEqual(self.report["map"]["resolution"], 0.1)
        self.assertEqual(self.report["map"]["width"], self.metadata["grid"]["width"])
        self.assertEqual(self.report["map"]["height"], self.metadata["grid"]["height"])
        self.assertEqual(self.report["map"]["resolution"], self.metadata["grid"]["resolution"])

    def test_current_content_hashes_are_computed_and_bound(self):
        pgm = DATA / "map.pgm"
        route = DATA / "hanyang_aegimun_loop.waypoints.yaml"
        pgm_sha256 = hashlib.sha256(pgm.read_bytes()).hexdigest()
        route_sha256 = hashlib.sha256(route.read_bytes()).hexdigest()
        self.assertEqual(pgm_sha256, "c89d791f71fe3d1705ae04724acf8ff6ba0ccc351fc162fe996982f9469a0278")
        self.assertEqual(route_sha256, "adf11b569c043da3b617f908ad56b2bc0ca6d32a32c6dd83a33a322045a4d672")
        self.assertEqual(self.report["map"]["sha256"], pgm_sha256)
        self.assertEqual(self.report["route"]["sha256"], route_sha256)
        self.assertEqual(self.metadata["hashes"]["pgm_sha256"], pgm_sha256)
        self.assertEqual(self.metadata["hashes"]["route_sha256"], route_sha256)
        self.assertEqual(self.route["map"]["sha256"], pgm_sha256)

    def test_trinary_pixel_classification_matches_map_metadata(self):
        map_document = VALIDATE.load_yaml(DATA / "map.yaml")
        classify = VALIDATE.classify_pixel
        negate = map_document["negate"]
        occupied_thresh = map_document["occupied_thresh"]
        free_thresh = map_document["free_thresh"]

        self.assertEqual(classify(192, negate, occupied_thresh, free_thresh), "free")
        self.assertEqual(classify(191, negate, occupied_thresh, free_thresh), "unknown")
        self.assertEqual(classify(89, negate, occupied_thresh, free_thresh), "occupied")
        self.assertEqual(classify(90, negate, occupied_thresh, free_thresh), "unknown")
        self.assertEqual(classify(0, 1, occupied_thresh, free_thresh), "free")
        self.assertEqual(classify(255, 1, occupied_thresh, free_thresh), "occupied")

        with self.assertRaises(ValueError):
            classify(192, 2, occupied_thresh, free_thresh)
        with self.assertRaises(ValueError):
            classify(192, negate, 0.2, 0.25)
    def test_current_assets_remain_candidate_only(self):
        self.assertTrue(self.report["valid"])
        self.assertTrue(self.report["candidate_qualified"])
        self.assertFalse(self.report["surveyed"])
        self.assertFalse(self.report["approved"])
        self.assertFalse(self.report["physically_qualified"])
        self.assertEqual(self.route["status"], "candidate")
        self.assertFalse(self.route["provenance"]["surveyed"])
        self.assertEqual(self.metadata["qualification"], "candidate")
        self.assertFalse(self.metadata["surveyed"])
        self.assertFalse(self.metadata["hardware_motion_authorized"])
        self.assertFalse(self.metadata["passenger_operation_authorized"])
        self.assertIn("candidate evidence is not a survey or approval", self.report["limitations"])

    def test_steep_grade_and_loop_blocker_are_explicit(self):
        grade = self.metadata["grade"]
        self.assertEqual(grade["formula"], "100*abs(dz)/sqrt(dx^2+dy^2)")
        self.assertEqual(grade["sample_count"], 3632)
        self.assertAlmostEqual(grade["max_grade_percent"], 25.2214, places=4)
        self.assertAlmostEqual(grade["p95_grade_percent"], 13.1412, places=4)
        self.assertAlmostEqual(grade["mean_grade_percent"], 5.3517, places=4)
        self.assertIsNone(self.report["grade_recomputed"], "runtime routes are intentionally planar")
        self.assertFalse(any(item.startswith("W_SUSPICIOUS_GRADE:") for item in self.report["warnings"]))

        closure = self.metadata["loop_closure"]
        self.assertAlmostEqual(closure["position_residual_m"], 0.9760259268134223)
        self.assertGreater(closure["position_residual_m"], closure["target_m"])
        self.assertFalse(closure["target_met"])
        self.assertFalse(self.metadata["candidate_qualification"]["loop_target_met"])

    def test_bidirectional_routes_are_navigable_without_runtime_reversal(self):
        self.assertEqual(self.report["route"]["pose_count"], 732)
        self.assertTrue(self.metadata["candidate_qualification"]["route_map_aligned"])
        self.assertEqual(self.metadata["grid"]["cleared_occupied_cells_in_recorded_corridor"], 2399)
        self.assertEqual(self.report["errors"], [])
        self.assertEqual(self.report["warnings"], [])

        expected = (("outbound_route", "outbound", 359), ("return_route", "return", 373))
        for key, direction, count in expected:
            route = self.route[key]
            self.assertEqual(route["direction"], direction)
            self.assertEqual(len(route["waypoints"]), count)
            for segment in route["segments"]:
                self.assertEqual(segment["max_linear_mps"], 0.0)
                self.assertEqual(segment["max_angular_rps"], 0.0)
                self.assertFalse(segment["hardware_authorized"])


if __name__ == "__main__":
    unittest.main()
