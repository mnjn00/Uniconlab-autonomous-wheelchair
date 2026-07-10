from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
NAV = ROOT / "src" / "wheelchair_navigation"


def _text(rel):
    return (NAV / rel).read_text()


def _scalar(text, key):
    match = re.search(rf"^\s*{re.escape(key)}:\s*([-+]?[0-9]*\.?[0-9]+)\s*$", text, re.MULTILINE)
    assert match, f"missing scalar key {key}"
    return float(match.group(1))


def test_navigation_launch_routes_move_base_to_raw_nav_topic_only():
    launch = NAV / "launch" / "navigation.launch"
    root = ET.parse(launch).getroot()
    move_base_nodes = [n for n in root.findall("node") if n.attrib.get("pkg") == "move_base"]
    assert len(move_base_nodes) == 1
    remaps = {(r.attrib.get("from"), r.attrib.get("to")) for r in move_base_nodes[0].findall("remap")}
    assert ("cmd_vel", "$(arg cmd_vel_nav_topic)") in remaps
    assert "/cmd_vel_safe" not in launch.read_text()


def test_move_base_uses_ros1_navigation_stack_components():
    text = _text("config/move_base.yaml")
    assert "base_global_planner: navfn/NavfnROS" in text
    assert "base_local_planner: dwa_local_planner/DWAPlannerROS" in text
    assert "controller_frequency: 10.0" in text
    forbidden = ["Nav2", "nav2_", "ros2_control", "launch_testing", "ros_gz", "gz_ros2_control"]
    for token in forbidden:
        assert token not in text


def test_costmap_and_local_planner_safety_thresholds_are_bounded():
    costmap = _text("config/costmap_common.yaml")
    dwa = _text("config/dwa_local_planner.yaml")
    assert "footprint: [[0.485, 0.300]" in costmap
    assert "/lidar/points" in costmap
    assert "/scan" in costmap
    assert _scalar(dwa, "max_vel_x") <= 0.70
    assert _scalar(dwa, "max_vel_trans") <= 0.70
    assert _scalar(dwa, "max_vel_theta") <= 1.00
    assert _scalar(dwa, "stop_time_buffer") >= 0.30


def test_geofence_configs_have_modes_polygons_and_clearance():
    for filename, mode in [
        ("geofence_sidewalk.yaml", "sidewalk"),
        ("geofence_road_free_space.yaml", "road_free_space"),
    ]:
        text = _text(f"config/{filename}")
        assert f"mode: {mode}" in text
        assert _scalar(text, "max_linear_speed") <= 0.70
        assert _scalar(text, "max_angular_speed") <= 1.00
        assert _scalar(text, "min_clearance_m") >= 0.35
        assert len(re.findall(r"^\s*- \[-?[0-9]", text, re.MULTILINE)) >= 4
