import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_ros_free_driver_covers_lifecycle_and_absolute_vetoes():
    completed = subprocess.run(
        ["python3", str(ROOT / "tools" / "static_threat_bypass_host_qa.py")],
        cwd=ROOT, check=False, capture_output=True, text=True, timeout=15,
    )

    assert completed.returncode == 0, completed.stderr
    records = tuple(json.loads(line) for line in completed.stdout.splitlines())
    cases = {record["case"]: record for record in records}
    required = {
        "person_wait_0_0", "person_wait_1_8", "person_commit_2_0",
        "object_wait_0_0", "object_wait_1_8", "object_commit_2_0",
        "safe_left_proposal", "stopped_target_turn_override",
        "accepted_zero_side_persistence",
        "single_dropout", "passing_behind", "tail_clear_1",
        "tail_clear_2", "tail_clear_3_release", "resume",
        "moving", "unknown", "learned_only", "changed_id",
        "second_dynamic_threat", "raw_only_blockage", "legacy_permit",
        "stale_permit", "proposal_tamper_rejected", "stale_proposal",
        "mismatched_proposal",
        "immediate_collision", "carried_collision", "proposal_collision",
        "localization_fault", "perception_fault", "odom_fault",
        "terrain_fault", "summary",
    }
    assert required <= cases.keys()
    assert all(record["passed"] is True for record in records)
    assert cases["safe_left_proposal"]["frame_id"] == "current_body"
    assert cases["safe_left_proposal"]["distance_m"] > 0.0
    assert cases["safe_left_proposal"]["latency_s"] > 0.0
    assert cases["safe_left_proposal"]["time_step_count"] > 1
    assert cases["stopped_target_turn_override"]["first_applied_w"] == 0.0
    assert cases["stopped_target_turn_override"]["target_w"] >= 0.08
    assert cases["resume"]["route_decision"] == "clear"
    assert cases["resume"]["command_v"] > 0.0
    assert cases["summary"]["result"] == "STATIC_THREAT_HOST_QA_PASS"


def test_host_and_qa_modes_are_read_only_and_publish_success_tokens():
    source = (ROOT / "tools" / "test_static_threat_bypass.sh").read_text()
    forbidden = ("roslaunch", "rosrun", "rosnode", "rostopic", "rsync", "git switch")

    assert "qa)" in source
    assert "STATIC_THREAT_HOST_TEST_PASS" in source
    assert "STATIC_THREAT_HOST_QA_PASS" in source
    assert all(value not in source for value in forbidden)

    help_result = subprocess.run(
        ["bash", str(ROOT / "tools" / "hybrid.sh"), "help"],
        cwd=ROOT, check=False, capture_output=True, text=True, timeout=5,
    )
    assert help_result.returncode == 0
    assert "static-threat-bypass-status" in help_result.stdout
