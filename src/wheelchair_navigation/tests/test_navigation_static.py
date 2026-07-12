import ast
import hashlib
import json
import math
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


def _boolean(text, key):
    match = re.search(rf"^\s*{re.escape(key)}:\s*(true|false)\s*$", text, re.MULTILINE)
    assert match, f"missing boolean key {key}"
    return match.group(1) == "true"


def _assert_finite_scalars(text, keys):
    for key in keys:
        assert math.isfinite(_scalar(text, key)), f"{key} must be finite"


def test_navigation_launch_routes_move_base_to_raw_nav_topic_only():
    launch = NAV / "launch" / "navigation.launch"
    root = ET.parse(launch).getroot()
    move_base_nodes = [n for n in root.findall("node") if n.attrib.get("pkg") == "move_base"]
    assert len(move_base_nodes) == 1
    remaps = [(r.attrib.get("from"), r.attrib.get("to")) for r in move_base_nodes[0].findall("remap")]
    command_remaps = [remap for remap in remaps if remap[0] == "cmd_vel"]
    assert command_remaps == [("cmd_vel", "$(arg cmd_vel_nav_topic)")]
    nav_topic_args = [a for a in root.findall("arg") if a.attrib.get("name") == "cmd_vel_nav_topic"]
    assert len(nav_topic_args) == 1
    assert nav_topic_args[0].attrib.get("default") == "/cmd_vel_nav"

    authority_text = launch.read_text() + _text("config/move_base.yaml")
    assert "/cmd_vel_safe" not in authority_text
    assert "/cmd_vel_nav" in launch.read_text()
    assert "hardware_motion_authorized: false" in authority_text
    assert "passenger_operation_authorized: false" in authority_text


def test_navigation_launch_binds_exactly_one_current_hanyang_map_server():
    launch = NAV / "launch" / "navigation.launch"
    root = ET.parse(launch).getroot()
    map_args = [item for item in root.findall("arg") if item.attrib.get("name") == "map_yaml"]
    assert len(map_args) == 1
    assert map_args[0].attrib["default"].endswith(
        "/../../data/hanyang_aegimun_loop/map.yaml"
    )
    servers = [node for node in root.findall("node") if node.attrib.get("pkg") == "map_server"]
    assert len(servers) == 1
    assert servers[0].attrib == {
        "pkg": "map_server",
        "type": "map_server",
        "name": "map_server",
        "args": "$(arg map_yaml)",
        "output": "screen",
        "required": "true",
    }
    map_yaml = ROOT / "data" / "hanyang_aegimun_loop" / "map.yaml"
    map_pgm = ROOT / "data" / "hanyang_aegimun_loop" / "map.pgm"
    assert hashlib.sha256(map_yaml.read_bytes()).hexdigest() == (
        "9f36ab2d3d35a667996570b45babb7cb3ba8bd1e706a971af22603da1220e7ff"
    )
    assert hashlib.sha256(map_pgm.read_bytes()).hexdigest() == (
        "c89d791f71fe3d1705ae04724acf8ff6ba0ccc351fc162fe996982f9469a0278"
    )
    assert "<exec_depend>map_server</exec_depend>" in (NAV / "package.xml").read_text()


def test_move_base_uses_ros1_navigation_stack_components():
    text = _text("config/move_base.yaml")
    assert "base_global_planner: navfn/NavfnROS" in text
    assert "base_local_planner: dwa_local_planner/DWAPlannerROS" in text
    assert "controller_frequency: 10.0" in text
    launch = ET.parse(NAV / "launch" / "navigation.launch").getroot()
    namespaces = {
        item.attrib["file"].rsplit("/", 1)[-1]: item.attrib.get("ns")
        for item in launch.findall("rosparam")
        if item.attrib.get("file")
    }
    assert namespaces["move_base.yaml"] == "move_base"
    assert namespaces["recovery_behaviors.yaml"] == "move_base"
    assert namespaces["global_costmap.yaml"] == "move_base"
    assert namespaces["local_costmap.yaml"] == "move_base"
    assert namespaces["dwa_local_planner.yaml"] == "move_base"
    common_namespaces = [
        item.attrib.get("ns")
        for item in launch.findall("rosparam")
        if item.attrib.get("file", "").endswith("/costmap_common.yaml")
    ]
    assert common_namespaces == [
        "move_base/global_costmap",
        "move_base/local_costmap",
    ]
    global_costmap = _text("config/global_costmap.yaml")
    assert _scalar(global_costmap, "resolution") == 0.10
    assert _scalar(text, "planner_frequency") == 0.0
    assert _boolean(global_costmap, "rolling_window")
    assert _scalar(global_costmap, "width") == 30.0
    assert _scalar(global_costmap, "height") == 30.0

    route_manifest = json.loads(
        (ROOT / "data" / "hanyang_aegimun_loop" /
         "hanyang_aegimun_loop.waypoints.yaml").read_text()
    )
    half_window = _scalar(global_costmap, "width") / 2.0
    for route_name in ("outbound_route", "return_route"):
        waypoints = route_manifest[route_name]["waypoints"]
        maximum_step = max(
            math.hypot(
                current["x_m"] - previous["x_m"],
                current["y_m"] - previous["y_m"],
            )
            for previous, current in zip(waypoints, waypoints[1:])
        )
        assert maximum_step < half_window
    forbidden = ["Nav2", "nav2_", "ros2_control", "launch_testing", "ros_gz", "gz_ros2_control"]
    for token in forbidden:
        assert token not in text


def test_wheelchair_footprint_padding_and_obstacle_cloud_contract():
    costmap = _text("config/costmap_common.yaml")
    footprint_match = re.search(r"^footprint:\s*(.+)$", costmap, re.MULTILINE)
    assert footprint_match
    footprint = ast.literal_eval(footprint_match.group(1))
    assert footprint == [
        [0.485, 0.300],
        [0.485, -0.300],
        [-0.485, -0.300],
        [-0.485, 0.300],
    ]
    assert max(point[0] for point in footprint) - min(point[0] for point in footprint) == 0.970
    assert max(point[1] for point in footprint) - min(point[1] for point in footprint) == 0.600
    assert 0.05 <= _scalar(costmap, "footprint_padding") <= 0.10
    assert _scalar(costmap, "transform_tolerance") <= 0.25
    assert "observation_sources: obstacle_cloud" in costmap
    assert "topic: /perception/obstacle_cloud" in costmap
    assert "sensor_frame: lidar_link" in costmap
    assert "data_type: PointCloud2" in costmap
    assert _boolean(costmap, "marking")
    assert _boolean(costmap, "clearing")
    assert 0.30 <= _scalar(costmap, "observation_persistence") <= 0.50
    assert 0.0 < _scalar(costmap, "expected_update_rate") <= 0.30
    assert _scalar(costmap, "raytrace_range") > _scalar(costmap, "obstacle_range")
    assert _scalar(costmap, "inflation_radius") >= 0.60


def test_dwa_is_forward_only_finite_and_prioritizes_route_adherence():
    dwa = _text("config/dwa_local_planner.yaml")
    finite_keys = [
        "max_vel_x",
        "min_vel_x",
        "max_vel_trans",
        "min_vel_trans",
        "max_vel_theta",
        "min_vel_theta",
        "acc_lim_x",
        "acc_lim_theta",
        "acc_lim_trans",
        "sim_time",
        "path_distance_bias",
        "goal_distance_bias",
        "occdist_scale",
        "stop_time_buffer",
    ]
    _assert_finite_scalars(dwa, finite_keys)
    assert _scalar(dwa, "min_vel_x") == 0.0
    assert _scalar(dwa, "min_vel_trans") == 0.0
    assert _scalar(dwa, "max_vel_y") == 0.0
    assert _scalar(dwa, "min_vel_y") == 0.0
    assert not _boolean(dwa, "holonomic_robot")
    assert _scalar(dwa, "max_vel_x") <= 0.55
    assert _scalar(dwa, "max_vel_trans") <= 0.55
    assert _scalar(dwa, "max_vel_theta") <= 0.85
    assert 0.0 < _scalar(dwa, "acc_lim_x") <= 0.50
    assert 0.0 < _scalar(dwa, "acc_lim_trans") <= 0.50
    assert _scalar(dwa, "stop_time_buffer") >= 0.50
    assert _scalar(dwa, "sim_time") >= 2.5
    assert _boolean(dwa, "penalize_negative_x")
    assert _scalar(dwa, "path_distance_bias") > _scalar(dwa, "goal_distance_bias")
    assert _scalar(dwa, "occdist_scale") >= 0.08
    collision_policy = (
        ROOT / "src" / "wheelchair_safety" / "config" / "collision_policy.yaml"
    ).read_text()
    controller_frequency = _scalar(_text("config/move_base.yaml"), "controller_frequency")
    assert (
        _scalar(dwa, "acc_lim_theta") / controller_frequency
        <= _scalar(collision_policy, "coverage_motion_angular_tolerance_rps")
    )


def test_unknown_space_and_unsafe_recovery_are_disabled():
    move_base = _text("config/move_base.yaml")
    assert "NavfnROS:" in move_base
    assert not _boolean(move_base, "allow_unknown")
    assert not _boolean(move_base, "clearing_rotation_allowed")
    assert not _boolean(move_base, "recovery_behavior_enabled")
    assert _scalar(move_base, "oscillation_timeout") > 0.0
    assert _scalar(move_base, "oscillation_distance") >= 0.10


def test_simulation_profiles_are_explicit_and_cannot_authorize_hardware():
    move_base = _text("config/move_base.yaml")
    assert "simulation_speed_profiles:" in move_base
    assert re.search(r"^\s{2}sidewalk:\s*$", move_base, re.MULTILINE)
    assert re.search(r"^\s{2}road_free_space:\s*$", move_base, re.MULTILINE)
    assert not _boolean(move_base, "hardware_motion_authorized")
    assert not _boolean(move_base, "passenger_operation_authorized")
    assert "geofence" not in move_base


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
