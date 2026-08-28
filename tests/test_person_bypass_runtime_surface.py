from pathlib import Path

ROOT = Path(__file__).parents[1]
PACKAGE = ROOT / "src" / "static_livox_localization"
SCRIPTS = PACKAGE / "scripts"


def text(path):
    return path.read_text(encoding="utf-8")


def test_catkin_installs_every_stationary_person_bypass_node():
    cmake = text(PACKAGE / "CMakeLists.txt")
    for name in (
        "person_bypass_policy.py",
        "person_bypass_dwa_follower.py",
        "person_bypass_semantic_supervisor.py",
        "trajectory_safety_gate.py",
        "person_bypass_preflight.py",
    ):
        assert name in cmake


def test_hybrid_start_activates_branch_and_go_refuses_old_graph():
    hybrid = text(ROOT / "tools" / "hybrid.sh")
    go = text(ROOT / "tools" / "go_hybrid.sh")
    activate = text(ROOT / "tools" / "activate_person_bypass.sh")
    assert '"$SCRIPT_DIR/activate_person_bypass.sh" activate' in hybrid
    assert "person_bypass_preflight.py" in go
    assert go.index("person_bypass_preflight.py") < go.index('exec "$GO"')
    assert "person_bypass_dwa_follower.py" in activate
    assert "person_bypass_semantic_supervisor.py" in activate
    assert "trajectory_safety_gate.py" in activate
    assert "__name:=waypoint_follower" in activate
    assert "__name:=semantic_safety_supervisor" in activate
    assert "__name:=safety_gate" in activate


def test_dwa_keeps_rtx_qualifies_while_paused_and_publishes_short_permit():
    follower = text(SCRIPTS / "person_bypass_dwa_follower.py")
    assert "install_gpu_planner(dwa_core)" in follower
    assert '"/person_bypass/permit"' in follower
    assert "StaticPersonQualifier" in follower
    assert "self.tracking_state == \"TRACKING\"" in follower
    assert "self.planner.max_speed" in follower
    assert "dwa_core.OBSTACLE_FLOOR_M" in follower
    assert "return GO_ROUND" in follower
    # Permit qualification happens before the inherited hold ladder can
    # return for PAUSED, otherwise a person already in front makes `go`
    # impossible forever.
    assert follower.index("if not self.enabled:") \
        < follower.index("super(PersonBypassDwaFollower, self).step()")


def test_semantic_exception_is_same_track_static_only():
    supervisor = text(SCRIPTS / "person_bypass_semantic_supervisor.py")
    policy = text(SCRIPTS / "person_bypass_policy.py")
    assert "permit_matches_observation" in supervisor
    assert "nearest_dynamic_threat" in supervisor
    assert "self.person_latch.reset()" in supervisor
    assert '"learned_only"' in policy
    assert "len(observations) != 1" in policy
    assert "PERSON_NOT_CONFIRMED_STATIC" in policy
    assert "LOCALIZATION_NOT_TRACKING" in policy


def test_raw_gate_replaces_only_fixed_corridor_obstacle_with_clear_curve():
    gate = text(SCRIPTS / "trajectory_safety_gate.py")
    assert 'if reason not in ("OBSTACLE", "OBSTACLE_SWEEP")' in gate
    assert "evaluate_gate_override" in gate
    assert "requested_path_collision" in gate
    assert "carried_path_collision" in gate
    assert "immediate_collision" in gate
    assert 'return "", decision.speed_cap_mps' in gate
    assert 'if reason != "OBSTACLE"' not in gate
    # SWEEP_MARGIN_M is already the protected current footprint. The branch
    # must not silently re-create the old ~0.75 m straight box with an extra
    # 0.10 m margin or a lower three-point threshold.
    assert '"~person_bypass_immediate_front_margin_m", 0.0' in gate
    assert '"~person_bypass_immediate_side_margin_m", 0.0' in gate
    assert '"~person_bypass_immediate_point_count", 5' in gate


def test_branch_preflight_proves_new_implementations_not_only_node_names():
    preflight = text(SCRIPTS / "person_bypass_preflight.py")
    assert "person_bypass_capable" in preflight
    assert "trajectory_person_bypass_capable" in preflight
    assert "permit_is_fresh" in preflight


def test_activation_passes_the_reliability_tunables_to_the_follower():
    activate = text(ROOT / "tools" / "activate_person_bypass.sh")
    hybrid = text(ROOT / "tools" / "hybrid.sh")

    assert 'PERSON_BYPASS_MAX_GAP_S="${PERSON_BYPASS_MAX_GAP_S:-0.45}"' \
        in activate
    assert "PERSON_BYPASS_LATERAL_HYSTERESIS_M" in activate
    assert "_person_bypass_lateral_hysteresis_m:" in activate
    assert 'PERSON_BYPASS_CLEARANCE_M="${PERSON_BYPASS_CLEARANCE_M:-0.35}"' \
        in activate
    follower = text(SCRIPTS / "person_bypass_dwa_follower.py")
    assert '"~person_bypass_clearance_m", 0.35' in follower
    assert "PERSON_BYPASS_CLEARANCE_M=0.35" in hybrid
    guard = text(SCRIPTS / "cluster_guard.py")
    assert "PERSON_BYPASS_CLEARANCE_M = 0.35" in guard


def test_success_profile_defaults_to_recorded_geometric_only_runtime():
    hybrid = text(ROOT / "tools" / "hybrid.sh")

    assert 'START_POINTPILLARS="${START_POINTPILLARS:-false}"' in hybrid
    assert 'REQUIRE_LEARNED="${REQUIRE_LEARNED:-false}"' in hybrid


def test_static_threat_test_entrypoint_is_non_driving_by_default():
    runner = text(ROOT / "tools" / "test_static_threat_bypass.sh")

    assert 'MODE="${1:-host}"' in runner
    assert "test_person_bypass_policy.py" in runner
    assert "test_dwa_policy.py" in runner
    assert "test_gpu_dwa_backend.py" in runner
    assert "test_python_node_packaging.py" in runner
    assert "person-bypass-status" in runner
    assert "hybrid.sh start" not in runner
    assert "hybrid.sh go" not in runner
