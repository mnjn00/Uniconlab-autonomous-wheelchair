"""What gets typed at the chair.

Three scripts, and each one's failure mode is different. trial_0727 has to
configure the run it claims to configure - a trial that quietly brings the
guards up measures nothing, because a band refusal and a lost fix are the
same stationary chair. go has to refuse rather than start, since anything it
cannot check is something that gets discovered while the chair is already
moving. stop has to work when everything else has failed, which means
checking nothing at all.
"""

from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
TOOLS = ROOT / "tools"


def script(name):
    return (TOOLS / name).read_text(encoding="utf-8")


def test_the_trial_switches_the_discretionary_guards_off():
    """That is the whole point of it: measure one thing without another
    guard ending the measurement first."""
    assert "export SAFETY_POLICIES=false" in script("trial_0727.sh")


def test_the_trial_leaves_object_avoidance_on():
    """~cluster_avoidance is deliberately not behind ~safety_policies, so the
    trial must not set it - and the bringup must not either."""
    assert "cluster_avoidance" not in script("trial_0727.sh")
    assert "cluster_avoidance" not in script("start_wheelchair_localization.sh")


def test_the_trial_does_not_drive():
    trial = script("trial_0727.sh")
    assert "waypoint_follower/start" not in trial
    assert "mode_cmd" not in trial


def test_priest_v5_trial_enables_the_guarded_runtime_profile() -> None:
    # Given the dedicated live-navigation bringup script.
    trial = script("trial_priest_v5.sh")

    # When its exported runtime profile is inspected.
    expected = (
        "export SAFETY_POLICIES=true",
        "export PLANNER=priest",
        "20260803_route_v5_waypoints.json",
        "20260803_route_v5_safety_band.json",
    )

    # Then it selects PRIEST over the latest band without starting motion.
    assert all(token in trial for token in expected)
    assert '"$SCRIPT_DIR/stop.sh"' in trial
    assert '"$SCRIPT_DIR/preflight_priest_v5.sh"' in trial
    assert "data: true" not in trial
    assert "std_msgs/Int16 65" not in trial


def test_priest_v5_preflight_proves_every_runtime_dependency() -> None:
    # Given the profile-specific preflight and generic bringup.
    preflight = script("preflight_priest_v5.sh")
    startup = script("start_wheelchair_localization.sh")

    # When the machine-consumed checks are enumerated.
    required = (
        "/waypoint_follower/planner",
        "/waypoint_follower/route",
        "/waypoint_follower/safety_band",
        "/waypoint_follower/safety_policies",
        "/safety_gate/safety_policies",
        "/perception/objects_summary",
        "/fast_lio_icp/localization_diagnostics",
        "/fast_lio_icp/auto_initialization_verified",
        "/tip_guard/status",
        "/cloud_registered_body",
        "/cmd_vel",
        "[r]osbag record",
    )

    # Then stale planner, global route, guards, sensors, or recorder refuse.
    assert all(token in preflight for token in required)
    assert '_planner:="$PLANNER"' in startup
    assert '[ -n "$LINEAR" ] && [ -n "$ANGULAR" ]' in preflight
    assert "data: true" not in preflight
    assert "std_msgs/Int16 65" not in preflight


def test_priest_v5_go_runs_preflight_before_any_motion_command() -> None:
    # Given the PRIEST-specific start command.
    go = script("go_priest_v5.sh")

    # When its command ordering is inspected.
    checked = go.index("preflight_priest_v5.sh")
    auto_mode = go.index("rostopic pub -1 /mode_cmd")
    start = go.index("waypoint_follower/start")

    # Then the complete preflight finishes before auto mode or follower start.
    assert checked < auto_mode < start


def test_priest_v5_scripts_are_bound_to_the_reviewed_deployment() -> None:
    # Given the laptop and NUC dirtiness gates plus the install loop.
    push = script("push_to_nuc.sh")
    local_start = push.index('DEPLOY_DIRTY="')
    local_gate = push[local_start:push.index(')"', local_start)]
    remote_start = push.index('REMOTE_DIRTY="')
    remote_gate = push[remote_start:push.index(')"', remote_start)]
    install_start = push.index("for script in ")
    install_loop = push[install_start:push.index("; do", install_start)]

    # When every installed field script is resolved in each exact surface.
    names = (
        "start_wheelchair_localization.sh",
        "trial_0727.sh",
        "trial_priest_v5.sh",
        "preflight_priest_v5.sh",
        "go.sh",
        "go_priest_v5.sh",
        "stop.sh",
    )

    # Then no uncommitted local or NUC copy can bypass the reviewed commit.
    for name in names:
        assert "tools/" + name in local_gate
        assert "tools/" + name in remote_gate
        assert name in install_loop


def test_go_refuses_when_the_follower_is_not_there():
    go = script("go.sh")
    assert "rosnode ping -c1 /waypoint_follower" in go
    assert "fail " in go


def test_go_refuses_on_a_silent_object_tracker():
    """It is the only guard still watching for people once the policies are
    off, and silence from it reads exactly like clear road."""
    go = script("go.sh")
    assert "/perception/objects_summary" in go


def test_go_refuses_unless_localization_is_tracking():
    go = script("go.sh")
    assert "localization_diagnostics" in go
    assert 'TRACKING' in go


def test_go_checks_everything_before_it_commands_anything():
    """A check after the first command is a check performed while the chair
    is already moving."""
    go = script("go.sh")
    last_check = max(go.index("/perception/objects_summary"),
                     go.index("localization_diagnostics"),
                     go.index("rosnode ping"))
    assert last_check < go.index("rostopic pub -1 /mode_cmd")
    assert last_check < go.index("waypoint_follower/start")


def test_stop_checks_nothing_and_tolerates_failure():
    """A stop that refuses because a precondition failed is not a stop."""
    stop = script("stop.sh")
    assert "set -e" not in stop
    assert stop.count("|| true") >= 2
    assert 'data: false' in stop


@pytest.mark.parametrize("name", [
    "trial_0727.sh",
    "trial_priest_v5.sh",
    "preflight_priest_v5.sh",
    "go.sh",
    "go_priest_v5.sh",
    "stop.sh",
])
def test_each_script_is_deployed_and_checksum_verified(name):
    """They live in $HOME, outside the checkout, so a pull never touches
    them - the same way the bringup was found still running the previous
    week's route."""
    push = script("push_to_nuc.sh")
    assert name in push
    assert "did not install cleanly" in push


@pytest.mark.parametrize("name", [
    "trial_0727.sh",
    "trial_priest_v5.sh",
    "preflight_priest_v5.sh",
    "go.sh",
    "go_priest_v5.sh",
    "stop.sh",
])
def test_each_script_is_executable(name):
    assert (TOOLS / name).stat().st_mode & 0o111, "%s is not executable" % name
