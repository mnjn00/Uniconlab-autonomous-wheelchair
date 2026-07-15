import importlib.util
from pathlib import Path
import unittest

MODULE_PATH = Path(__file__).parents[1] / "tools" / "static_localization_preflight.py"
SPEC = importlib.util.spec_from_file_location("static_localization_preflight", str(MODULE_PATH))
module = importlib.util.module_from_spec(SPEC)

class PreflightPolicyTest(unittest.TestCase):
    def test_rejects_any_command_topic_publisher(self):
        SPEC.loader.exec_module(module)
        self.assertFalse(module.is_safe_graph({"/cmd_vel": ["/unsafe"]}, []))

    def test_rejects_external_map_to_odom_authority(self):
        SPEC.loader.exec_module(module)
        self.assertFalse(module.is_safe_graph({}, ["/amcl"]))

    def test_accepts_empty_command_graph_and_no_external_authority(self):
        SPEC.loader.exec_module(module)
        self.assertTrue(module.is_safe_graph({}, []))

if __name__ == "__main__":
    unittest.main()
