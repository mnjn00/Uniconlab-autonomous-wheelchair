from pathlib import Path

ROOT = Path(__file__).parents[1]


def test_start_wrapper_keeps_original_stack_as_rollback():
    text = (ROOT / 'tools/start_hybrid_avoidance.sh').read_text()
    assert 'PROFILE=dwa' in text
    assert 'start_wheelchair_localization.sh' in text
    assert '_cmd_topic:=/cmd_vel_planned' in text
    assert '/cmd_vel_gated:=/cmd_vel_terrain_safe' in text


def test_geometric_summary_is_remapped_before_fusion():
    text = (ROOT / 'tools/start_hybrid_avoidance.sh').read_text()
    assert '/perception/objects_summary:=/perception/geometric_objects_summary' in text
    assert 'hybrid_object_fusion.py' in text


def test_new_nodes_are_installed_by_catkin():
    cmake = (ROOT / 'src/static_livox_localization/CMakeLists.txt').read_text()
    for name in (
        'hybrid_object_fusion.py', 'vision_detection_bridge.py',
        'semantic_safety_supervisor.py', 'terrain_guard.py',
        'hybrid_preflight.py', 'hybrid_perception.py',
        'semantic_safety_policy.py', 'terrain_guard_policy.py',
    ):
        assert name in cmake


def test_go_checks_everything_before_delegating_to_go():
    text = (ROOT / 'tools/go_hybrid.sh').read_text()
    check = text.index('hybrid_preflight.py')
    delegate = text.index('exec "$GO"')
    assert check < delegate
