"""Regression tests for the explicit Livox ICP candidate source."""

import importlib.util
from pathlib import Path
import unittest
import sys

MODULE_PATH = Path(__file__).parents[1] / "scripts" / "localization_adapter.py"
SPEC = importlib.util.spec_from_file_location("localization_adapter", str(MODULE_PATH))
adapter = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = adapter
SPEC.loader.exec_module(adapter)

class FastLioIcpSourceTest(unittest.TestCase):
    def test_fast_lio_icp_is_the_only_enabled_source(self):
        self.assertEqual(adapter.select_native_source("fast_lio_icp", ("fast_lio_icp",)), "fast_lio_icp")

    def test_fast_lio_icp_is_listed_as_supported(self):
        self.assertIn("fast_lio_icp", adapter.VALID_SOURCES)

    def test_pose_sequence_must_match_diagnostic_reset_epoch(self):
        self.assertTrue(
            adapter.pose_reset_binding_matches("fast_lio_icp", 3, 3)
        )
        self.assertFalse(
            adapter.pose_reset_binding_matches("fast_lio_icp", 2, 3)
        )
        self.assertFalse(
            adapter.pose_reset_binding_matches("fast_lio_icp", 0, None)
        )

if __name__ == "__main__":
    unittest.main()
