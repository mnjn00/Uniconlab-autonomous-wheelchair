import importlib.util
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = (
    Path(__file__).parents[1] / "scripts" / "localization_preflight.py"
)
SPEC = importlib.util.spec_from_file_location(
    "localization_preflight", MODULE_PATH
)
preflight = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preflight)


class PreflightTest(unittest.TestCase):
    def test_acceleration_unit_classifier_accepts_both_driver_conventions(self):
        self.assertEqual(
            preflight.classify_acceleration_norm(0.9994), "G_UNITS"
        )
        self.assertEqual(
            preflight.classify_acceleration_norm(9.80665), "MPS2_UNITS"
        )
        self.assertEqual(
            preflight.classify_acceleration_norm(3.0), "UNEXPECTED"
        )

    def test_map_identity_is_exact(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.ply"
            path.write_bytes(b"ply\n")
            digest = preflight.sha256_file(path)
            preflight.validate_map(path, digest)
            with self.assertRaisesRegex(ValueError, "mismatch"):
                preflight.validate_map(path, "0" * 64)


if __name__ == "__main__":
    unittest.main()
