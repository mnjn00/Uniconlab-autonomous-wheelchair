"""Static ownership and source-selection checks for navigation perception."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "src" / "wheelchair_perception" / "launch" / "perception.launch"
NODE = ROOT / "src" / "wheelchair_perception" / "scripts" / "perception_node.py"


class PerceptionLaunchStaticTests(unittest.TestCase):
    def setUp(self):
        self.launch_text = LAUNCH.read_text(encoding="utf-8")
        self.node_text = NODE.read_text(encoding="utf-8")
        self.root = ET.fromstring(self.launch_text)

    def test_profiles_have_explicit_cloud_and_imu_remaps(self):
        expected = {
            "simulation": ("$(arg simulation_cloud_topic)", "$(arg simulation_imu_topic)"),
            "replay": ("$(arg replay_cloud_topic)", "$(arg replay_imu_topic)"),
            "hardware_shadow": ("$(arg hardware_cloud_topic)", "$(arg hardware_imu_topic)"),
        }
        groups = self.root.findall("group")
        self.assertEqual(len(groups), 3)
        for profile, topics in expected.items():
            group = next(item for item in groups if "'{}'".format(profile) in item.attrib["if"])
            nodes = group.findall("node")
            self.assertEqual(len(nodes), 1)
            self.assertEqual(nodes[0].attrib["required"], "true")
            remaps = {item.attrib["from"]: item.attrib["to"] for item in nodes[0].findall("remap")}
            self.assertEqual(remaps, {"input_cloud": topics[0], "input_imu": topics[1]})

    def test_every_input_is_latest_only_and_defaults_are_conservative(self):
        for node in self.root.findall("group/node"):
            params = {item.attrib["name"]: item.attrib["value"] for item in node.findall("param")}
            self.assertEqual(params["input_queue_size"], "1")
            self.assertEqual(params["max_cloud_age_s"], "$(arg max_cloud_age_s)")
            self.assertEqual(params["imu_alignment_tolerance_s"], "$(arg imu_alignment_tolerance_s)")
        self.assertIn('name="max_cloud_age_s" default="0.30"', self.launch_text)
        self.assertIn('name="hardware_cloud_topic" default="/hardware/unconfigured/', self.launch_text)
        self.assertIn('name="hardware_imu_topic" default="/hardware/unconfigured/', self.launch_text)
        self.assertIn("queue_size=1", self.node_text)

    def test_node_owns_only_navigation_products(self):
        self.assertIn('"/perception/obstacle_cloud"', self.node_text)
        self.assertIn('"/perception/diagnostics"', self.node_text)
        forbidden = (
            'Publisher("/cmd_vel',
            "Publisher('/cmd_vel",
            'Publisher("/safety/',
            "Publisher('/safety/",
            "SafetySignal",
            "map->odom",
            "rclpy",
            "ros2",
        )
        for token in forbidden:
            with self.subTest(token=token):
                self.assertNotIn(token, self.node_text)
                self.assertNotIn(token, self.launch_text)

    def test_ros_imports_are_lazy(self):
        main_offset = self.node_text.index("def main():")
        for token in ("import rospy", "from sensor_msgs.msg", "from diagnostic_msgs.msg"):
            self.assertGreater(self.node_text.index(token), main_offset)


if __name__ == "__main__":
    unittest.main()
