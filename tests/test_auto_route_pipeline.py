"""Static checks for the drop-safe auto-route planning pipeline.

These verify the launch wiring, the costmap config, and the script packaging
without a ROS master - the same approach test_navigation_static.py takes.
"""

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT / "src" / "wheelchair_navigation"
TOOLS = ROOT / "tools"


def _text(rel):
    return (NAV / rel).read_text()


def test_auto_planner_launch_serves_dropsafe_map_and_navfn():
    launch = NAV / "launch" / "auto_planner.launch"
    root = ET.parse(launch).getroot()
    map_servers = [n for n in root.findall("node") if n.attrib.get("pkg") == "map_server"]
    assert len(map_servers) == 1
    assert "dropsafe" in map_servers[0].attrib.get("args", "")
    move_base = [n for n in root.findall("node") if n.attrib.get("pkg") == "move_base"]
    assert len(move_base) == 1
    # Planning-only: cmd_vel is remapped away from anything the follower chain uses.
    remaps = {(r.attrib.get("from"), r.attrib.get("to"))
              for r in move_base[0].findall("remap")}
    assert ("cmd_vel", "/cmd_vel_nav_unused") in remaps
    # The drop-safe global costmap is loaded, not the obstacle-bearing common one.
    loaded = [p.attrib.get("file", "") for p in root.findall("rosparam")
              if p.attrib.get("command") == "load"]
    assert any("dropsafe_global_costmap" in s for s in loaded)


def test_dropsafe_global_costmap_has_no_obstacle_layer():
    text = _text("config/dropsafe_global_costmap.yaml")
    assert "obstacle_layer" not in text
    assert "StaticLayer" in text
    assert "track_unknown_space: true" in text


def test_move_base_navfn_disallows_unknown():
    text = _text("config/move_base.yaml")
    assert re.search(r"allow_unknown:\s*false", text)


def test_make_plan_client_is_installed():
    cmake = (NAV / "CMakeLists.txt").read_text()
    assert "make_plan_client.py" in cmake


def test_bake_dropsafe_costmap_and_path_to_route_assets_exist():
    assert (TOOLS / "bake_dropsafe_costmap.py").is_file()
    assert (TOOLS / "path_to_route_assets.py").is_file()


def test_dropsafe_pgm_origin_matches_grid():
    """The pgm origin is the lower-left world coordinate, flipped on write."""
    src = (TOOLS / "bake_dropsafe_costmap.py").read_text()
    assert "image[::-1]" in src
    assert "origin: [%.4f, %.4f, 0.0]" in src or "origin: [" in src


def test_path_to_route_assets_emits_chair_centre_reference():
    src = (TOOLS / "path_to_route_assets.py").read_text()
    assert '"reference_point": "chair_centre"' in src
    assert '"body_frame_profile"' in src
