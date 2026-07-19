import importlib.util
from pathlib import Path


PATH = Path(__file__).parents[1] / "scripts" / "preflight.py"
SPEC = importlib.util.spec_from_file_location("preflight", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def publishers():
    return {topic: ["/driver"] for topic in MODULE.REQUIRED_TOPICS}


def test_valid_graph_and_map(tmp_path):
    path = tmp_path / "map.pcd"
    path.write_bytes(b"pcd")
    code, _ = MODULE.evaluate(publishers(), str(path), MODULE.sha256_file(path))
    assert code == 0


def test_missing_sensor_is_rejected(tmp_path):
    path = tmp_path / "map.pcd"
    path.write_bytes(b"pcd")
    graph = publishers(); graph["/livox/imu"] = []
    assert MODULE.evaluate(graph, str(path), MODULE.sha256_file(path))[0] == 10


def test_motion_command_publisher_is_rejected(tmp_path):
    path = tmp_path / "map.pcd"
    path.write_bytes(b"pcd")
    graph = publishers(); graph["/cmd_vel"] = ["/controller"]
    assert MODULE.evaluate(graph, str(path), MODULE.sha256_file(path))[0] == 12
