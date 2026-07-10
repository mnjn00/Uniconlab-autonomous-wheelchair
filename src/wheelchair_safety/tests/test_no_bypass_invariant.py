from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]

LAUNCH_AND_CONFIG = [
    *ROOT.glob("src/*/launch/*.launch"),
    *ROOT.glob("src/*/config/*.yaml"),
]


def test_move_base_output_is_not_wired_directly_to_base_controller():
    offenders = []
    unsafe_sources = ["/cmd_vel_nav", "/cmd_vel_raw", '"/cmd_vel"', " /cmd_vel "]
    base_command_topics = ["/wheelchair_base_controller/cmd_vel", "/base_controller/cmd_vel"]
    for path in LAUNCH_AND_CONFIG:
        for idx, line in enumerate(path.read_text().splitlines(), start=1):
            stripped = line.strip()
            if stripped.startswith("<arg "):
                continue
            if any(topic in line for topic in base_command_topics):
                if "/cmd_vel_safe" not in line:
                    offenders.append(f"{path}:{idx}: {stripped}")
                for unsafe in unsafe_sources:
                    if unsafe in line and "/cmd_vel_safe" not in line:
                        offenders.append(f"{path}:{idx}: unsafe source {unsafe}: {stripped}")
    assert offenders == []


def test_expected_nav_to_safety_to_base_command_chain_is_present():
    nav_launch = ROOT / "src" / "wheelchair_navigation" / "launch" / "navigation.launch"
    safety_launch = ROOT / "src" / "wheelchair_safety" / "launch" / "safety.launch"
    sim_bringup = ROOT / "src" / "wheelchair_bringup" / "launch" / "sim_bringup.launch"

    nav_root = ET.parse(nav_launch).getroot()
    move_base = [n for n in nav_root.findall("node") if n.attrib.get("pkg") == "move_base"][0]
    remaps = {(r.attrib.get("from"), r.attrib.get("to")) for r in move_base.findall("remap")}
    assert ("cmd_vel", "$(arg cmd_vel_nav_topic)") in remaps
    assert 'default="/cmd_vel_nav"' in nav_launch.read_text()

    safety_text = safety_launch.read_text()
    assert 'default="/cmd_vel_nav"' in safety_text
    assert 'default="/cmd_vel_safe"' in safety_text

    sim_text = sim_bringup.read_text()
    assert "/cmd_vel_safe /wheelchair_base_controller/cmd_vel" in sim_text
    assert "/cmd_vel_nav /wheelchair_base_controller/cmd_vel" not in sim_text
    assert "/cmd_vel_raw /wheelchair_base_controller/cmd_vel" not in sim_text
