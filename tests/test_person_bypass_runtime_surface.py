import importlib.util
import json
import dataclasses
from pathlib import Path
import subprocess
import sys
from types import ModuleType, SimpleNamespace

import pytest


ROOT = Path(__file__).parents[1]
PACKAGE = ROOT / "src" / "static_livox_localization"
SCRIPTS = PACKAGE / "scripts"


def text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def runtime_sources() -> str:
    paths = (
        ROOT / "tools" / "activate_person_bypass.sh",
        ROOT / "tools" / "hybrid.sh",
        SCRIPTS / "person_bypass_dwa_follower.py",
        SCRIPTS / "person_bypass_semantic_supervisor.py",
        SCRIPTS / "trajectory_safety_gate.py",
        SCRIPTS / "person_bypass_preflight.py",
    )
    return "\n".join(text(path) for path in paths)


def load_preflight(monkeypatch, *, semantic_capable: bool = True,
                   confirmation_s: float = 2.0):
    class RosException(RuntimeError):
        pass

    class String:
        def __init__(self, data: str = ""):
            self.data = data

    permit = json.dumps({
        "schema": "static-threat-bypass/v2",
        "capable": True,
        "active": False,
        "stamp": 100.0,
        "expires": 100.45,
        "track_id": None,
        "target_x_m": None,
        "target_y_m": None,
        "threat_label": None,
        "static_for_s": 0.0,
        "max_speed_mps": 0.35,
        "min_clearance_m": 0.35,
        "reason": "NO_STATIC_THREAT",
    }, separators=(",", ":"), sort_keys=True)
    topics = {
        "/static_threat_bypass/permit": permit,
        "/semantic_safety/status": json.dumps({
            "static_threat_bypass_capable": semantic_capable,
        }),
        "/safety_gate/status": json.dumps({
            "static_threat_bypass_capable": True,
            "static_threat_bypass_proposal_capable": True,
        }),
    }
    parameters = {
        "/waypoint_follower/static_threat_bypass_capable": True,
        "/waypoint_follower/static_threat_bypass_proposal_capable": True,
        "/waypoint_follower/static_threat_bypass_confirmation_s":
            confirmation_s,
        "/semantic_safety_supervisor/static_threat_bypass_capable": True,
        "/safety_gate/static_threat_bypass_capable": True,
        "/safety_gate/static_threat_bypass_proposal_capable": True,
    }
    rospy = ModuleType("rospy")
    rospy.ROSException = RosException
    rospy.init_node = lambda *args, **kwargs: None
    rospy.get_param = lambda name, default=None: parameters.get(name, default)
    rospy.wait_for_message = (
        lambda topic, _type, timeout: String(topics[topic]))
    rospy.Time = SimpleNamespace(
        now=lambda: SimpleNamespace(to_sec=lambda: 100.1))
    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg
    monkeypatch.setitem(sys.modules, "rospy", rospy)
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)
    spec = importlib.util.spec_from_file_location(
        "runtime_surface_preflight", SCRIPTS / "person_bypass_preflight.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_catkin_installs_static_threat_nodes_and_proposal_contract():
    # Given the catkin runtime package
    cmake = text(PACKAGE / "CMakeLists.txt")
    # When its installed Python files are enumerated
    required = (
        "person_bypass_policy.py", "trajectory_proposal.py",
        "person_bypass_dwa_follower.py",
        "person_bypass_semantic_supervisor.py",
        "trajectory_safety_gate.py", "person_bypass_preflight.py",
    )
    # Then every node and sibling wire contract is packaged.
    assert all(name in cmake for name in required)


def test_proposal_contract_imports_with_python38_dataclass(monkeypatch):
    # Given the Python 3.8 dataclass API used by the NUC
    native_dataclass = dataclasses.dataclass

    def python38_dataclass(*args, **kwargs):
        assert "slots" not in kwargs, \
            "Python 3.8 dataclass does not accept slots"
        return native_dataclass(*args, **kwargs)

    monkeypatch.setattr(dataclasses, "dataclass", python38_dataclass)
    spec = importlib.util.spec_from_file_location(
        "python38_trajectory_proposal", SCRIPTS / "trajectory_proposal.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)

    # When the proposal contract is imported
    spec.loader.exec_module(module)

    # Then its strict value types are available without Python 3.10 features.
    assert module.TrajectoryProposal.__name__ == "TrajectoryProposal"


def test_runtime_surface_uses_only_generic_static_threat_contract():
    # Given all active user-facing bypass surfaces
    source = runtime_sources()
    # When names are compared with the strict generic contract
    required = (
        "/static_threat_bypass/permit", "/static_threat_bypass/proposal",
        "STATIC_THREAT_BYPASS_CONFIRM_S",
        "~static_threat_bypass_confirmation_s",
        "~static_threat_bypass_capable",
        "~static_threat_bypass_proposal_capable",
    )
    legacy = (
        "/person_bypass/permit", "PERSON_BYPASS_", "~person_bypass_",
        "_person_bypass_", '"person_bypass_capable"',
        '"trajectory_person_bypass_capable"', '"person_bypass_track_id"',
    )
    # Then no legacy parameter, topic, capability, or diagnostic key remains.
    assert all(value in source for value in required)
    assert all(value not in source for value in legacy)


def test_static_threat_confirmation_default_is_exactly_two_seconds():
    # Given the shell launcher and follower parameter boundary
    activate = text(ROOT / "tools" / "activate_person_bypass.sh")
    follower = text(SCRIPTS / "person_bypass_dwa_follower.py")
    # When defaults are inspected
    shell_default = (
        'STATIC_THREAT_BYPASS_CONFIRM_S="'
        '${STATIC_THREAT_BYPASS_CONFIRM_S:-2.0}"')
    # Then both runtime entry points use the same exact two-second default.
    assert shell_default in activate
    assert '"~static_threat_bypass_confirmation_s", 2.0' in follower


def test_preflight_is_read_only_and_checks_v2_proposal_capability():
    # Given the field preflight executable
    preflight = text(SCRIPTS / "person_bypass_preflight.py")
    # When its readiness checks are inspected
    required = (
        "PERMIT_SCHEMA", "static_threat_bypass_capable",
        "static_threat_bypass_proposal_capable",
        "static_threat_bypass_confirmation_s", "permit_is_fresh",
    )
    mutating = (
        "rospy.set_param", "rospy.Publisher", "rospy.ServiceProxy",
        "rosnode", "rosservice", "subprocess", "os.system",
    )
    # Then it proves strict v2 readiness without changing runtime state.
    assert all(value in preflight for value in required)
    assert all(value not in preflight for value in mutating)


def test_preflight_accepts_strict_v2_generic_capabilities(monkeypatch, capsys):
    # Given a strict v2 permit and every generic capability
    preflight = load_preflight(monkeypatch)
    # When the read-only readiness check runs
    preflight.main()
    # Then it reports the machine-readable generic readiness token.
    assert "STATIC_THREAT_BYPASS_PREFLIGHT_OK" in capsys.readouterr().out


def test_preflight_rejects_legacy_or_missing_generic_capability(monkeypatch):
    # Given a graph without the semantic generic capability
    preflight = load_preflight(monkeypatch, semantic_capable=False)
    # When readiness is evaluated
    with pytest.raises(preflight.Failure, match="stop-only"):
        preflight.main()
    # Then the legacy-compatible graph is not accepted.


def test_preflight_rejects_nonexact_confirmation_window(monkeypatch):
    # Given an otherwise capable graph configured for 2.1 seconds
    preflight = load_preflight(monkeypatch, confirmation_s=2.1)
    # When readiness is evaluated
    with pytest.raises(preflight.Failure, match="exactly 2.0"):
        preflight.main()
    # Then field activation remains fail-closed.


def test_blackbox_records_every_bypass_and_command_chain_surface():
    # Given the activation recorder
    activate = text(ROOT / "tools" / "activate_person_bypass.sh")
    # When the rosbag command is inspected
    topics = (
        "/static_threat_bypass/permit", "/static_threat_bypass/proposal",
        "/static_threat_bypass/status", "/waypoint_follower/status",
        "/semantic_safety/status", "/safety_gate/status",
        "/terrain_guard/status", "/tip_guard/status",
        "/cmd_vel_planned", "/cmd_vel_raw", "/cmd_vel_gated",
        "/cmd_vel_terrain_safe", "/cmd_vel", "/wheel_cmd",
        "/wheel_status", "/mode_cmd",
    )
    # Then policy, proposal, every command stage, and wheel feedback are kept.
    assert all(topic in activate for topic in topics)


def test_hybrid_help_exposes_generic_two_second_configuration_only():
    # Given the host-safe CLI help command
    command = ["bash", str(ROOT / "tools" / "hybrid.sh"), "help"]
    # When a user invokes it without ROS
    completed = subprocess.run(
        command, check=False, capture_output=True, text=True, timeout=5)
    # Then the generic configuration is observable and no legacy knob leaks.
    assert completed.returncode == 0, completed.stderr
    assert "STATIC_THREAT_BYPASS_CONFIRM_S=2.0" in completed.stdout
    assert "PERSON_BYPASS_" not in completed.stdout


def test_host_runtime_checks_never_start_stop_or_connect_to_ros():
    # Given the host-only runtime surface tests
    source = text(Path(__file__))
    # When live-control invocations are searched
    forbidden = (
        "rosnode" + " kill", "ros" + "launch", "ros" + "run",
        "ros" + "topic", "hybrid.sh" + " start", "hybrid.sh" + " go",
    )
    # Then the test suite cannot drive, deploy, or mutate a ROS graph.
    assert all(value not in source for value in forbidden)
