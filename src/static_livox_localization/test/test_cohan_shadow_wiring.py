"""Machine contracts for the isolated CoHAN/HATEB shadow graph."""

import ast
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PACKAGE = Path(__file__).parents[1]
SCRIPTS = PACKAGE / "scripts"
ROOT = PACKAGE.parents[1]


def publisher_topics(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    topics = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if not (
            isinstance(function, ast.Attribute)
            and function.attr == "Publisher"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[0].value, str)
        ):
            continue
        topics.append(node.args[0].value)
    return topics


def test_shadow_adapter_has_no_motion_authority_publisher():
    # Given: the concrete ROS adapter source that will run beside HATEB.
    node = SCRIPTS / "human_aware_shadow_node.py"
    assert node.exists(), "no shadow ROS adapter exists"

    # When: every statically declared publisher topic is extracted.
    topics = publisher_topics(node)

    # Then: only advisory human/status surfaces exist.
    assert set(topics) == {
        "/human_aware_shadow/tracked_agents",
        "/human_aware_shadow/status",
    }
    forbidden = {
        "/cmd_vel_raw",
        "/cmd_vel_gated",
        "/cmd_vel",
        "/wheel_cmd",
        "/mode_cmd",
    }
    assert forbidden.isdisjoint(topics)


def test_shadow_metadata_pins_official_cohan_contract():
    # Given: the machine-consumed external integration manifest.
    path = PACKAGE / "config" / "cohan_shadow.yaml"
    assert path.exists(), "no pinned CoHAN shadow metadata exists"

    # When: its upstream and topic contracts are parsed.
    config = yaml.safe_load(path.read_text(encoding="utf-8"))

    # Then: the audited revision and advisory boundary are immutable.
    assert config["upstream"] == {
        "repository": "https://github.com/LAAS-HRI/CoHAN2.0.git",
        "commit": "bdfc5240b2128347bf6fe501d392d14768754a9d",
    }
    assert config["tracked_agents_topic"] == \
        "/human_aware_shadow/tracked_agents"
    assert config["velocity_sink_topic"] == \
        "/human_aware_shadow/velocity_proposal"
    assert config["planner_plugin"] == \
        "hateb_local_planner/HATebLocalPlannerROS"


def test_shadow_launch_sinks_every_hateb_velocity_command():
    # Given: the isolated launch graph.
    path = PACKAGE / "launch" / "cohan_shadow.launch"
    assert path.exists(), "no isolated CoHAN shadow launch exists"

    # When: launch nodes, params, and remaps are parsed.
    root = ET.fromstring(path.read_text(encoding="utf-8"))
    args = root.findall("./arg")
    nodes = root.findall(".//node")
    remaps = root.findall(".//remap")
    params = root.findall(".//param")
    rosparams = root.findall(".//rosparam")

    # Then: HATEB is namespaced and its velocity reaches only the sink.
    assert any(
        node.get("pkg") == "static_livox_localization"
        and node.get("type") == "human_aware_shadow_node.py"
        for node in nodes
    )
    assert any(
        node.get("pkg") == "agent_path_prediction"
        and node.get("type") == "agent_path_predict"
        for node in nodes
    )
    assert any(
        node.get("pkg") == "move_base"
        and node.get("type") == "move_base"
        for node in nodes
    )
    assert any(
        param.get("name") == "base_local_planner"
        and param.get("value")
        == "hateb_local_planner/HATebLocalPlannerROS"
        for param in params
    )
    assert any(
        param.get("name") == "goals_file"
        and param.get("value")
        == "$(find agent_path_prediction)/cfg/goals_adream.yaml"
        for param in params
    )
    assert any(
        arg.get("name") == "broadcast_robot_tf"
        and arg.get("default") == "false"
        for arg in args
    )
    assert any(
        param.get("name") == "broadcast_robot_tf"
        and param.get("value") == "$(arg broadcast_robot_tf)"
        for param in params
    )
    assert any(
        param.get("name") == "local_costmap/global_frame"
        and param.get("value") == "$(arg global_frame)"
        for param in params
    )
    for namespace in ("global_costmap", "local_costmap"):
        assert any(
            param.get("name") == f"{namespace}/obstacle_layer/enabled"
            and param.get("value") == "false"
            for param in params
        )
    assert any(
        remap.get("from") == "cmd_vel"
        and remap.get("to")
        == "/human_aware_shadow/velocity_proposal"
        for remap in remaps
    )
    assert not any(
        remap.get("to") in {
            "/cmd_vel_raw",
            "/cmd_vel_gated",
            "/cmd_vel",
            "/wheel_cmd",
            "/mode_cmd",
        }
        for remap in remaps
    )
    cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "scripts/human_aware_shadow_node.py" in cmake
    assert "scripts/human_aware_shadow.py" in cmake
    common = [
        item for item in rosparams
        if item.get("file", "").endswith("/config/costmap_common.yaml")
    ]
    assert {item.get("ns") for item in common} == {
        "global_costmap",
        "local_costmap",
    }
    hateb = yaml.safe_load(
        (PACKAGE / "config" / "cohan_shadow_hateb.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert hateb["odom_topic"] == "/Odometry"
    assert hateb["max_vel_x_backwards"] > hateb["penalty_epsilon"]
    runner = (ROOT / "tools" / "run_cohan_shadow_replay.sh").read_text(
        encoding="utf-8"
    )
    assert "broadcast_robot_tf:=true" in runner


def test_replay_validator_imports_under_python38_builtins():
    # Given: Python 3.8, where builtin collections are not subscriptable.
    path = ROOT / "tools" / "validate_cohan_shadow_replay.py"
    spec = importlib.util.spec_from_file_location("cohan_replay_validator", path)
    module = importlib.util.module_from_spec(spec)

    class Python38Dict:
        pass

    module.__dict__["dict"] = Python38Dict

    # When/Then: importing the shipped CLI must not evaluate dict[...] at runtime.
    spec.loader.exec_module(module)


def test_replay_runner_loads_isolated_native_dependencies_before_hateb():
    runner = (ROOT / "tools" / "run_cohan_shadow_replay.sh").read_text(
        encoding="utf-8"
    )
    assert 'COHAN_LOCAL_DEPS_ROOT="${COHAN_LOCAL_DEPS_ROOT:-' in runner
    assert "openblas-pthread" in runner
    assert 'ldd "$COHAN_WS/devel/lib/libhateb_local_planner.so"' in runner


def test_replay_goal_is_nearby_instead_of_route_endpoint(tmp_path):
    path = ROOT / "tools" / "select_cohan_shadow_goal.py"
    spec = importlib.util.spec_from_file_location("cohan_goal_selector", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    waypoints = [{"x": float(index), "y": 0.0} for index in range(20)]
    assert module.select_goal(waypoints, 0.1, 0.0) == (5.0, 0.0)
    pose_csv = tmp_path / "pose.csv"
    pose_csv.write_text(
        "%time,field.pose.pose.position.x,field.pose.pose.position.y\n"
        "1000000000,-0.4,0.15\n",
        encoding="utf-8",
    )
    assert module.pose_from_csv(pose_csv) == (-0.4, 0.15)

    runner = (ROOT / "tools" / "run_cohan_shadow_replay.sh").read_text(
        encoding="utf-8"
    )
    assert "select_cohan_shadow_goal.py" in runner
    assert '["waypoints"][-1]' not in runner
