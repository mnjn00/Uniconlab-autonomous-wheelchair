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

if __name__ == "__main__":
    unittest.main()
