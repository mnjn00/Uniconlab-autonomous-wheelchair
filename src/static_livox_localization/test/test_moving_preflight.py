import importlib.util
from pathlib import Path


PATH = Path(__file__).parents[1] / "scripts" / "preflight.py"
SPEC = importlib.util.spec_from_file_location("moving_preflight", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def valid_publishers():
    return {topic: [f"/publisher{index}"] for index, topic in enumerate(MODULE.REQUIRED_TOPICS)}


def valid_map(tmp_path):
    path = tmp_path / "map.pcd"
    path.write_bytes(b"fixed-map")
    return path, MODULE.sha256_file(path)


def test_rejects_duplicate_required_topic_publishers(tmp_path):
    path, digest = valid_map(tmp_path)
    publishers = valid_publishers()
    publishers["/livox/lidar"] = ["/driver_a", "/driver_b"]

    code, message = MODULE.evaluate(
        publishers, str(path), digest,
        topic_ages={topic: 0.0 for topic in MODULE.REQUIRED_TOPICS},
        tf_authorities=[],
    )

    assert code == 10
    assert "duplicate" in message


def test_rejects_required_topic_without_fresh_message(tmp_path):
    path, digest = valid_map(tmp_path)
    ages = {topic: 0.0 for topic in MODULE.REQUIRED_TOPICS}
    ages["/livox/imu"] = 2.5

    code, message = MODULE.evaluate(
        valid_publishers(), str(path), digest,
        topic_ages=ages, max_topic_age_s=1.0, tf_authorities=[],
    )

    assert code == 10
    assert "stale" in message


def test_rejects_existing_map_to_odom_tf_authority(tmp_path):
    path, digest = valid_map(tmp_path)

    code, message = MODULE.evaluate(
        valid_publishers(), str(path), digest,
        topic_ages={topic: 0.0 for topic in MODULE.REQUIRED_TOPICS},
        tf_authorities=["/old_localizer"],
    )

    assert code == 13
    assert "/old_localizer" in message


def test_accepts_fresh_single_publishers_and_no_tf_conflict(tmp_path):
    path, digest = valid_map(tmp_path)

    code, message = MODULE.evaluate(
        valid_publishers(), str(path), digest,
        topic_ages={topic: 0.0 for topic in MODULE.REQUIRED_TOPICS},
        tf_authorities=[],
    )

    assert code == 0
    assert message == "preflight passed"

