from pathlib import Path


ROOT = Path(__file__).parents[1]


def text(path):
    return path.read_text(encoding="utf-8")


def test_legacy_profile_restores_field_geometric_defaults():
    profile = text(ROOT / "tools" / "perception_profile.sh")

    legacy = profile.split("legacy_geometric)", 1)[1].split(";;", 1)[0]
    assert '${START_POINTPILLARS:=false}' in legacy
    assert '${GEOMETRIC_FIXED_MAP_SUBTRACTION:=false}' in legacy
    assert '${GEOMETRIC_MIN_CELL_POINTS:=2}' in legacy
    assert '${GEOMETRIC_MIN_CLUSTER_POINTS:=8}' in legacy
    assert '${GEOMETRIC_MAX_CLUSTERS:=40}' in legacy
    assert '${GEOMETRIC_ROI_X_MIN_M:=-0.30}' in legacy
    assert '${GEOMETRIC_FORWARD_FOV_HALF_DEG:=115}' in legacy


def test_bringup_and_go_source_one_perception_profile():
    start = text(ROOT / "tools" / "start_hybrid_avoidance.sh")
    go = text(ROOT / "tools" / "go_hybrid.sh")

    assert '. "$SCRIPT_DIR/perception_profile.sh"' in start
    assert '. "$SCRIPT_DIR/perception_profile.sh"' in go
    assert 'START_POINTPILLARS="${START_POINTPILLARS:-true}"' not in start
    assert 'START_POINTPILLARS="${START_POINTPILLARS:-true}"' not in go


def test_geometric_profile_is_wired_to_the_runtime_node():
    start = text(ROOT / "tools" / "start_hybrid_avoidance.sh")
    producer = text(
        ROOT
        / "src"
        / "static_livox_localization"
        / "scripts"
        / "hybrid_geometric_objects.py"
    )

    assert '_fixed_map_subtraction:="$GEOMETRIC_FIXED_MAP_SUBTRACTION"' in start
    assert '_roi_x_min_m:="$GEOMETRIC_ROI_X_MIN_M"' in start
    assert '_forward_fov_half_deg:="$GEOMETRIC_FORWARD_FOV_HALF_DEG"' in start
    assert '_bool_param("fixed_map_subtraction", False)' in producer
    assert '_float_param("roi_x_min_m", legacy.ROI_X[0])' in producer
    assert '"forward_fov_half_deg", legacy.FORWARD_FOV_HALF_DEG)' in producer


def test_pointpillars_sparse_frame_floor_matches_field_override():
    config = text(
        ROOT
        / "src"
        / "static_livox_localization"
        / "config"
        / "pointpillars_rtx2060.yaml"
    )
    assert "minimum_points: 350" in config


def test_gate_feedback_changes_remain_in_integrated_branch():
    activate = text(ROOT / "tools" / "activate_person_bypass.sh")
    dwa = text(
        ROOT
        / "src"
        / "static_livox_localization"
        / "scripts"
        / "dwa_follower.py"
    )

    assert "_accepted_cmd_topic:=/cmd_vel" in activate
    assert '"~accepted_cmd_topic", "/cmd_vel"' in dwa
    assert "self.on_accepted_command" in dwa
