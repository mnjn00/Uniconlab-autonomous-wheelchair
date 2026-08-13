"""Static safety contract for exact-commit non-driving NUC deployment."""

from pathlib import Path


SCRIPT = (
    Path(__file__).parents[3]
    / "tools"
    / "deploy_and_run_nuc_shadow_qa.sh"
).read_text(encoding="utf-8")


def test_deployment_verifies_exact_host_and_commit():
    assert "PINNED_KEY=" in SCRIPT
    assert 'ssh-keygen -lf "$KNOWN_HOSTS"' in SCRIPT
    assert "git fetch origin relax/tracking-thresholds" in SCRIPT
    assert 'git archive "$EXPECTED_COMMIT"' in SCRIPT
    assert 'git merge --ff-only' not in SCRIPT
    assert 'git checkout' not in SCRIPT


def test_deployment_refuses_motion_and_runs_shadow_gate():
    motion_check = SCRIPT.index('pgrep -af "\\$AUTONOMOUS_RE"')
    deploy = SCRIPT.index('rsync -a --delete')
    shadow = SCRIPT.index('"\\$DEPLOY/tools/run_nuc_shadow_qa.sh"')
    assert motion_check < deploy < shadow
    assert "wheel.launch" not in SCRIPT
    assert "waypoint_follower.py" not in SCRIPT
    assert "dwa_follower.py" not in SCRIPT
    assert "mpc_follower.py" not in SCRIPT


def test_shadow_workspace_never_mutates_field_package_or_build():
    assert 'FIELD_WS="\\$HOME/livox_static_localization_ws"' in SCRIPT
    assert 'WS="\\$HOME/.cache/unicon-shadow-$EXPECTED_COMMIT"' in SCRIPT
    assert '"\\$FIELD_WS/src/static_livox_localization/"' not in SCRIPT
    assert 'mkdir -p "\\$WS/src"' in SCRIPT
    assert 'catkin config --extend "\\$FIELD_WS/devel"' in SCRIPT
    assert '"\\$DEPLOY/src/static_livox_localization/"' in SCRIPT
    assert '"\\$WS/src/static_livox_localization/"' in SCRIPT
