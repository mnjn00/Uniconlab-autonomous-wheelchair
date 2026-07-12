import ast
from pathlib import Path


ROOT = Path(__file__).parents[1]
MISSION = ROOT / "src" / "wheelchair_decision" / "scripts" / "mission_node.py"
LAUNCH = ROOT / "src" / "wheelchair_decision" / "launch" / "decision.launch"


def test_decision_has_no_direct_motion_command_surface():
    source = MISSION.read_text(encoding="utf-8")
    tree = ast.parse(source)
    forbidden_type = "Twi" + "st"
    forbidden_topic = "/cmd" + "_vel"

    for item in ast.walk(tree):
        if isinstance(item, ast.ImportFrom):
            assert all(alias.name != forbidden_type for alias in item.names)
        if isinstance(item, ast.Constant) and isinstance(item.value, str):
            assert forbidden_topic not in item.value

    launch = LAUNCH.read_text(encoding="utf-8")
    assert forbidden_topic not in launch
    assert forbidden_type not in launch


def test_decision_only_orchestrates_move_base_and_bounded_evidence():
    source = MISSION.read_text(encoding="utf-8")
    assert 'SimpleActionClient("move_base", MoveBaseAction)' in source
    assert 'Publisher("/route/active", ActiveRoute, queue_size=1, latch=False)' in source
    assert 'Publisher("/mission/state", MissionState, queue_size=1, latch=False)' in source
    assert 'Publisher("/decision/motion_intent", MotionIntent, queue_size=1, latch=False)' in source
    assert "Publisher(\"/route/progress\"" not in source
